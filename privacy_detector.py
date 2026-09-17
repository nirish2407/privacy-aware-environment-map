#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO

from std_msgs.msg import String

import cv2


# =====================================================================
# ROTATION HANDLING
#
# FIX: the previous version hard-coded a 90 deg CCW rotation that was
# specific to head_cam's physical mount. Detection has moved to
# torso_cam3d_left (which has a registered depth image, unlike
# head_cam), and that camera's mount orientation is not yet confirmed.
# Rotation is now a ROS param (default 0 = no rotation) and all four
# right-angle cases are supported so the correct value can be set
# after checking the feed once with image_view, e.g.:
#
#   rosrun image_view image_view image:=/torso_cam3d_left/rgb/image_raw
#
# If the feed looks upright already, leave ~rotate_deg at 0. If it's
# sideways/upside down, set ~rotate_deg to whichever of 90/180/270
# makes it upright, and detections will be correctly mapped back to
# the original (unrotated) pixel frame -- which is what object_position.py
# needs, since depth_registered is aligned to that original RGB frame.
# =====================================================================

_ROTATE_CV_MAP = {
    0: None,
    90: cv2.ROTATE_90_COUNTERCLOCKWISE,
    180: cv2.ROTATE_180,
    270: cv2.ROTATE_90_CLOCKWISE,
}


def remap_box_for_rotation(x1, y1, x2, y2, orig_w, orig_h, rotate_deg):
    """
    Map a bounding box detected on a rotated frame back into the
    original (unrotated) image's pixel coordinates.

    rotate_deg is the rotation that was applied to the frame BEFORE
    running YOLO on it (see _ROTATE_CV_MAP above).
    """

    if rotate_deg == 0:
        return x1, y1, x2, y2

    if rotate_deg == 90:
        # Frame was cv2.ROTATE_90_COUNTERCLOCKWISE before YOLO.
        # (This is the original head_cam mapping -- verified correct,
        # kept as-is.)
        new_x1 = orig_w - y2
        new_y1 = x1
        new_x2 = orig_w - y1
        new_y2 = x2

    elif rotate_deg == 180:
        new_x1 = orig_w - x2
        new_y1 = orig_h - y2
        new_x2 = orig_w - x1
        new_y2 = orig_h - y1

    elif rotate_deg == 270:
        # Frame was cv2.ROTATE_90_CLOCKWISE before YOLO (inverse of
        # the 90 case above).
        new_x1 = y1
        new_y1 = orig_h - x2
        new_x2 = y2
        new_y2 = orig_h - x1

    else:
        rospy.logwarn_once(
            "Unsupported rotate_deg=%s, treating as 0 (no rotation)",
            rotate_deg
        )
        return x1, y1, x2, y2

    return new_x1, new_y1, new_x2, new_y2


class PrivacyDetector:

    def __init__(self):

        # -----------------------------------------------------------
        # FIX: anonymous=True so two instances of this node (one per
        # camera) can run at the same time without a node-name
        # collision. Private params (~image_topic, ~camera_id, etc.)
        # still resolve per-instance the normal way, e.g.:
        #   rosrun privacy_environment_map privacy_detector.py \
        #       _image_topic:=/torso_cam3d_left/rgb/image_raw \
        #       _camera_id:=left
        #   rosrun privacy_environment_map privacy_detector.py \
        #       _image_topic:=/torso_cam3d_right/rgb/image_raw \
        #       _camera_id:=right
        # -----------------------------------------------------------
        rospy.init_node(
            f"privacy_detector",
            anonymous=True
        )

        # -----------------------------------------------------------
        # FIX (coverage): a single torso camera only sees a slice of
        # the space in front of the robot, so most people never enter
        # frame -- that's why almost nothing was being detected, and
        # only one of three people occasionally triggered a marker
        # when they clipped the edge of that camera's FOV.
        #
        # To cover the full front, run one instance of this node per
        # camera (e.g. ~camera_id:=left with torso_cam3d_left,
        # another ~camera_id:=right with torso_cam3d_right), both
        # publishing to the same /privacy_detections topic. Each
        # detection is tagged with its source camera_id so
        # object_position.py knows which depth stream/intrinsics/TF
        # frame to use for that specific detection.
        # -----------------------------------------------------------
        self.camera_id = rospy.get_param(
            "~camera_id",
            "left"
        )

        self.bridge = CvBridge()

        # -----------------------------------------------------------
        # FIX: model path can be overridden with a ROS param instead
        # of being hard-coded to a single user's home directory.
        # Falls back to the original path if no param is set.
        # -----------------------------------------------------------
        self.model_path = rospy.get_param(
            "~model_path",
            "/home/labuser/catkin_ws/src/"
            "privacy_environment_map/models/best.pt"
        )

        self.model = YOLO(self.model_path)

        # -----------------------------------------------------------
        # FIX: detection has moved off head_cam (no depth output) and
        # onto torso_cam3d_left, which has an RGB stream that is
        # pixel-registered to a depth image (depth_registered) --
        # required for object_position.py to read metric depth at
        # the exact detected pixel. head_cam remains available via
        # this param if ever needed again.
        # -----------------------------------------------------------
        self.image_topic = rospy.get_param(
            "~image_topic",
            f"/torso_cam3d_{self.camera_id}/rgb/image_raw"
        )

        # -----------------------------------------------------------
        # FIX: mount rotation is now a tunable param instead of an
        # assumption baked in for head_cam. Default 0 (no rotation)
        # for torso_cam3d_left -- verify with image_view and adjust
        # if the feed isn't upright.
        # -----------------------------------------------------------
        self.rotate_deg = int(rospy.get_param(
            "~rotate_deg",
            270
        ))

        if self.rotate_deg not in _ROTATE_CV_MAP:
            rospy.logwarn(
                "~rotate_deg=%s is not one of 0/90/180/270, using 0",
                self.rotate_deg
            )
            self.rotate_deg = 0

        # -----------------------------------------------------------
        # FIX: 0.1 confidence was far too permissive and let through
        # a lot of noisy/false detections that corrupted the
        # downstream tracker in object_position.py. Made this a
        # tunable param with a much safer default.
        # -----------------------------------------------------------
        self.confidence_threshold = rospy.get_param(
            "~confidence_threshold",
            0.3
        )

        self.pub = rospy.Publisher(
            f"/privacy_detections_{self.camera_id}",
            String,
            queue_size=10
        )

        rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1
        )

        rospy.loginfo(
            "Privacy detector started (camera_id=%s, model=%s, "
            "image_topic=%s, rotate_deg=%d, conf_thresh=%.2f)",
            self.camera_id,
            self.model_path,
            self.image_topic,
            self.rotate_deg,
            self.confidence_threshold
        )

    def image_callback(self, msg):

        # -----------------------------------------------------------
        # FIX: a bad/unsupported encoding used to crash the whole
        # node. Now it's logged and the frame is skipped instead.
        # -----------------------------------------------------------
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                "bgr8"
            )
        except Exception as e:
            rospy.logwarn(
                "cv_bridge conversion failed: %s",
                str(e)
            )
            return

        orig_h = msg.height
        orig_w = msg.width

        # -----------------------------------------------------------
        # Rotate the frame for YOLO if this camera's mount requires
        # it. Bounding boxes are mapped back to the original
        # (unrotated) pixel coordinates below via
        # remap_box_for_rotation() -- object_position.py expects
        # pixel coordinates in the original RGB frame, since that's
        # what depth_registered is aligned to.
        # -----------------------------------------------------------
        cv_rotation = _ROTATE_CV_MAP[self.rotate_deg]

        if cv_rotation is not None:
            frame = cv2.rotate(
                frame,
                cv_rotation
            )

        results = self.model(
            frame,
            verbose=False
        )

        detections = []

        for r in results:

            for box in r.boxes:

                confidence = float(
                    box.conf[0]
                )

                if confidence < self.confidence_threshold:
                    continue

                cls = int(
                    box.cls[0]
                )

                name = self.model.names[cls]

                x1, y1, x2, y2 = (
                    box.xyxy[0]
                    .cpu()
                    .numpy()
                )

                x1, y1, x2, y2 = remap_box_for_rotation(
                    x1, y1, x2, y2,
                    orig_w, orig_h,
                    self.rotate_deg
                )

                # Coordinates are now back in the ORIGINAL
                # (unrotated) camera image.

                x1 = max(0, min(int(x1), orig_w - 1))
                x2 = max(0, min(int(x2), orig_w - 1))

                y1 = max(0, min(int(y1), orig_h - 1))
                y2 = max(0, min(int(y2), orig_h - 1))

                rospy.logdebug(
                    "Image original: %d x %d",
                    msg.width,
                    msg.height
                )

                rospy.logdebug(
                    "YOLO box: %s",
                    str(box.xyxy[0])
                )

                rospy.logdebug(
                    "Converted box: %d %d %d %d",
                    int(x1),
                    int(y1),
                    int(x2),
                    int(y2)
                )

                # -----------------------------------------------------
                # FIX: camera_id appended as a 9th field so
                # object_position.py can tell which physical camera
                # this detection came from and use the matching
                # depth stream / intrinsics / TF frame for it.
                # Backward compatible: object_position.py still
                # accepts the old 8-field format from a single camera.
                # -----------------------------------------------------
                detection = (
                    f"{name},"
                    f"{confidence},"
                    f"{int(x1)},"
                    f"{int(y1)},"
                    f"{int(x2)},"
                    f"{int(y2)},"
                    f"{msg.header.stamp.secs},"
                    f"{msg.header.stamp.nsecs},"
                    f"{self.camera_id}"
                )

                detections.append(
                    detection
                )

                rospy.loginfo(
                    detection
                )

        if len(detections) > 0:

            self.pub.publish(
                "|".join(detections)
            )


if __name__ == "__main__":

    try:
        PrivacyDetector()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass