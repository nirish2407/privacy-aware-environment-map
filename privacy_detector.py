#!/usr/bin/env python3

# ROS Python client library.
import rospy

# ROS message type for camera images.
from sensor_msgs.msg import Image

# CvBridge converts ROS Image messages into OpenCV images.
from cv_bridge import CvBridge

# Ultralytics YOLO object-detection framework.
from ultralytics import YOLO

# ROS String message is used to publish detection information
from std_msgs.msg import String

# OpenCV is used for image manipulation, including rotation.
import cv2

# =====================================================================
# ROTATION HANDLING
#
# The camera image may not always be physically mounted in the same
# orientation as the robot's expected coordinate system.
#
# The previous version hard-coded a 90 deg CCW rotation that was specific
# to head_cam's physical mount. Detection has moved to torso_cam3d_left
# and torso_cam3d_right (which has a registered depth image, unlike head_cam).
# Rotation is now a ROS param (default 0 = no rotation) and all four right-angle
# cases are supported so the correct value can be set after checking the feed
# once with image_view, e.g.:
#
#      rosrun image_view image_view image:=/torso_cam3d_left/rgb/image_raw
#
# If the feed looks upright already, leave ~rotate_deg at 0. If it's
# sideways/upside down, set ~rotate_deg to whichever of 90/180/270
# makes it upright, and detections will be correctly mapped back to
# the original (unrotated) pixel frame -- which is what object_position.py
# needs, since depth_registered is aligned to that original RGB frame.
#
# Rotation is configurable using the ROS parameter:
#
#     ~rotate_deg
#
# Supported values:
#
#     0   -> no rotation
#     90  -> 90 degrees counter-clockwise
#     180 -> 180 degrees
#     270 -> 90 degrees clockwise
#
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

    Parameters:
        x1, y1:
            Top-left corner of the bounding box detected by YOLO.

        x2, y2:
            Bottom-right corner of the bounding box detected by YOLO.

        orig_w:
            Width of the original camera image.

        orig_h:
            Height of the original camera image.

        rotate_deg:
            Rotation applied to the image before YOLO inference.

    Returns:
        x1, y1, x2, y2:
            Bounding-box coordinates in the original image.
    
    """

    # If no rotation was applied, the coordinates are already correct.
    if rotate_deg == 0:
        return x1, y1, x2, y2

    # ---------------------------------------------------------------
    # 90-degree counter-clockwise rotation.
    #
    # The image was rotated using:
    #
    #     cv2.ROTATE_90_COUNTERCLOCKWISE
    #
    # Convert the YOLO coordinates back to the original image.
    # ---------------------------------------------------------------
    if rotate_deg == 90:
        # Frame was cv2.ROTATE_90_COUNTERCLOCKWISE before YOLO.
        new_x1 = orig_w - y2
        new_y1 = x1
        new_x2 = orig_w - y1
        new_y2 = x2

    # ---------------------------------------------------------------
    # 180-degree rotation.
    # ---------------------------------------------------------
    elif rotate_deg == 180:
        new_x1 = orig_w - x2
        new_y1 = orig_h - y2
        new_x2 = orig_w - x1
        new_y2 = orig_h - y1

    # ---------------------------------------------------------------
    # 270-degree rotation.
    #
    # 270 degrees is equivalent to a 90-degree clockwise rotation.
    # This is the inverse orientation of the 90-degree case above.
    # ---------------------------------------------------------------
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

    # Return the bounding box in the original image coordinate frame.
    return new_x1, new_y1, new_x2, new_y2


class PrivacyDetector:

    def __init__(self):

        # =============================================================
        # ROS NODE INITIALIZATION
        # =============================================================

        # -----------------------------------------------------------
        # anonymous=True so two instances of this node (one per
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

        # =============================================================
        # CAMERA IDENTIFICATION
        # =============================================================

        # -----------------------------------------------------------
        # a single torso camera only sees a slice of the space in
        # front of the robot, so most people never enter frame
        # that's why almost nothing was being detected, and
        # only one of three people occasionally triggered a marker
        # when they clipped the edge of that camera's FOV.
        #
        # To cover the full front, run one instance of this node per
        # camera (e.g. ~camera_id:=left with torso_cam3d_left,
        # another ~camera_id:=right with torso_cam3d_right), both
        # publishing to their respective /privacy_detections topic. Each
        # detection is tagged with its source camera_id so
        # object_position.py knows which depth stream/intrinsics/TF
        # frame to use for that specific detection.
        # -----------------------------------------------------------
        self.camera_id = rospy.get_param(
            "~camera_id",
            "left"
        )

        # Create a CvBridge instance.
        self.bridge = CvBridge()

        # =============================================================
        # YOLO MODEL
        # =============================================================

        # Get the YOLO model path from the ROS parameter server.
        #
        # This avoids hard-coding the model location and allows the
        # model to be changed when starting the node.
        #
        # Example:
        #
        #     _model_path:=/path/to/my_model.pt
        #
        # If no parameter is supplied, the default model path below
        # is used.
        self.model_path = rospy.get_param(
            "~model_path",
            "/home/labuser/catkin_ws/src/"
            "privacy_environment_map/models/best.pt"
        )

        # Load the trained YOLO model.
        self.model = YOLO(self.model_path)

        # =============================================================
        # CAMERA IMAGE TOPIC
        # =============================================================

        # Get the ROS topic from which camera images will be received.
        #
        # By default, the topic is constructed using camera_id:
        #
        #     camera_id = left
        #
        # becomes:
        #
        #     /torso_cam3d_left/rgb/image_raw
        #
        # and:
        #
        #     camera_id = right
        #
        # becomes:
        #
        #     /torso_cam3d_right/rgb/image_raw
        #
        # This can also be overridden manually with:
        #
        #     _image_topic:=/some/other/topic
        #
        self.image_topic = rospy.get_param(
            "~image_topic",
            f"/torso_cam3d_{self.camera_id}/rgb/image_raw"
        )

        # =============================================================
        # IMAGE ROTATION
        # =============================================================

        # Get the camera rotation from the ROS parameter server.
        #
        # The default is currently 270 degrees.
        #
        # IMPORTANT:
        # The correct value depends on the physical mounting
        # orientation of the camera.
        self.rotate_deg = int(rospy.get_param(
            "~rotate_deg",
            270
        ))

        # Make sure the rotation value is supported.
        if self.rotate_deg not in _ROTATE_CV_MAP:
            rospy.logwarn(
                "~rotate_deg=%s is not one of 0/90/180/270, using 0",
                self.rotate_deg
            )

            # Fall back to no rotation if an invalid value was supplied.
            self.rotate_deg = 0

        # =============================================================
        # DETECTION CONFIDENCE
        # =============================================================

        # Minimum confidence required for a YOLO detection to be
        # accepted.
        #
        # A detection with:
        #
        #     confidence < confidence_threshold
        #
        # is ignored.
        #
        # The default value is 0.3.
        #
        # This is configurable using:
        #
        #     _confidence_threshold:=0.5
        #
        # for example.
        self.confidence_threshold = rospy.get_param(
            "~confidence_threshold",
            0.3
        )

        # =============================================================
        # ROS PUBLISHER
        # =============================================================

        # Create the publisher used to send detections to downstream
        # nodes.
        #
        # Each camera has its own topic:
        #
        #     /privacy_detections_left
        #
        #     /privacy_detections_right
        #
        # This allows multiple camera detector instances to operate
        # independently.
        self.pub = rospy.Publisher(
            f"/privacy_detections_{self.camera_id}",
            String,
            queue_size=10
        )

        # =============================================================
        # ROS IMAGE SUBSCRIBER
        # =============================================================

        # Subscribe to the configured camera image topic.
        #
        # Whenever a new image arrives, ROS calls:
        #
        #     self.image_callback()
        #
        # queue_size=1 ensures that the detector does not build up a
        # large backlog of old camera frames if YOLO takes longer to
        # process an image than the camera takes to produce one.
        rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1
        )

        # Print the detector configuration when the node starts.
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

        # =============================================================
        # CONVERT ROS IMAGE TO OPENCV IMAGE
        # =============================================================

        # -----------------------------------------------------------
        # a bad/unsupported encoding used to crash the whole
        # node. Now it's logged and the frame is skipped instead.
        # -----------------------------------------------------------

        # Convert the incoming ROS Image message into an OpenCV BGR
        # image that can be passed to YOLO.
        #
        # If the image encoding is unsupported or conversion fails,
        # catch the exception instead of allowing the entire ROS node
        # to crash.
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

        # Store the dimensions of the ORIGINAL image.
        orig_h = msg.height
        orig_w = msg.width

        # =============================================================
        # IMAGE ROTATION
        # =============================================================

        # -----------------------------------------------------------
        # Rotate the frame for YOLO if this camera's mount requires
        # it. Bounding boxes are mapped back to the original
        # (unrotated) pixel coordinates below via
        # remap_box_for_rotation() -- object_position.py expects
        # pixel coordinates in the original RGB frame, since that's
        # what depth_registered is aligned to.
        # -----------------------------------------------------------

        # Look up the corresponding OpenCV rotation operation for the
        # configured rotation angle.
        cv_rotation = _ROTATE_CV_MAP[self.rotate_deg]

        # If a rotation is configured, rotate the image before sending
        # it to YOLO.
        if cv_rotation is not None:
            frame = cv2.rotate(
                frame,
                cv_rotation
            )

        # =============================================================
        # YOLO INFERENCE
        # =============================================================

        # Run object detection on the image.
        #
        # verbose=False prevents YOLO from printing its own detailed
        # inference information for every frame.
        results = self.model(
            frame,
            verbose=False
        )

        # This list will contain all valid detections from this frame.
        detections = []

        # =============================================================
        # PROCESS YOLO RESULTS
        # =============================================================

        # YOLO may return multiple result objects.
        for r in results:

            # Each result may contain multiple detected bounding boxes.
            for box in r.boxes:

                # -----------------------------------------------------
                # DETECTION CONFIDENCE
                # -----------------------------------------------------

                # Extract the confidence score from the YOLO result.
                #
                # Example:
                #
                #     0.87 -> 87% confidence
                confidence = float(
                    box.conf[0]
                )

                # Ignore detections below the configured confidence
                # threshold.
                if confidence < self.confidence_threshold:
                    continue

                # -----------------------------------------------------
                # CLASS IDENTIFICATION
                # -----------------------------------------------------

                # Get the numerical class ID predicted by YOLO.
                cls = int(
                    box.cls[0]
                )

                # Convert the numerical class ID into the human-readable
                # class name using the model's class-name mapping.
                name = self.model.names[cls]

                # -----------------------------------------------------
                # BOUNDING BOX
                # -----------------------------------------------------

                # Extract the YOLO bounding box coordinates.
                #
                # YOLO returns:
                #
                #     x1 = left
                #     y1 = top
                #     x2 = right
                #     y2 = bottom
                #
                # The coordinates are initially relative to the image
                # that was passed to YOLO, which may have been rotated.
                x1, y1, x2, y2 = (
                    box.xyxy[0]
                    .cpu()
                    .numpy()
                )

                # -----------------------------------------------------
                # MAP BOX BACK TO ORIGINAL IMAGE
                # -----------------------------------------------------

                # If the image was rotated before inference, convert
                # the YOLO bounding box back into the coordinate system
                # of the ORIGINAL camera image.
                #
                # This is essential for depth lookup because
                # depth_registered is aligned with the original RGB
                # image.
                x1, y1, x2, y2 = remap_box_for_rotation(
                    x1, y1, x2, y2,
                    orig_w, orig_h,
                    self.rotate_deg
                )

                # -----------------------------------------------------
                # CLAMP BOUNDING BOX TO IMAGE BOUNDARIES
                # -----------------------------------------------------

                # Coordinates are now back in the ORIGINAL
                # (unrotated) camera image.

                # Ensure x coordinates stay within:
                #
                #     0 <= x < image_width
                #
                x1 = max(0, min(int(x1), orig_w - 1))
                x2 = max(0, min(int(x2), orig_w - 1))

                # Ensure y coordinates stay within:
                #
                #     0 <= y < image_height
                #
                y1 = max(0, min(int(y1), orig_h - 1))
                y2 = max(0, min(int(y2), orig_h - 1))

                # -----------------------------------------------------
                # DEBUG INFORMATION
                # -----------------------------------------------------

                # Print the original image dimensions when DEBUG
                # logging is enabled.
                rospy.logdebug(
                    "Image original: %d x %d",
                    msg.width,
                    msg.height
                )

                # Print the bounding box as originally returned by YOLO.
                rospy.logdebug(
                    "YOLO box: %s",
                    str(box.xyxy[0])
                )

                # Print the bounding box after conversion back to the
                # original camera coordinate system.
                rospy.logdebug(
                    "Converted box: %d %d %d %d",
                    int(x1),
                    int(y1),
                    int(x2),
                    int(y2)
                )

                # =====================================================
                # BUILD DETECTION MESSAGE
                # =====================================================

                # -----------------------------------------------------
                # camera_id appended as a 9th field so that the 
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

                # Add this detection to the list for the current frame.
                detections.append(
                    detection
                )

                # Log the detection.
                rospy.loginfo(
                    detection
                )

        # =============================================================
        # PUBLISH DETECTIONS
        # =============================================================

        # Only publish when at least one valid detection was found.
        if len(detections) > 0:

            # Multiple detections are joined using "|"
            self.pub.publish(
                "|".join(detections)
            )

# =====================================================================
# MAIN PROGRAM
# =====================================================================

if __name__ == "__main__":

    try:

        # Create the PrivacyDetector object.
        PrivacyDetector()

        # Keep the node alive and allow ROS callbacks
        rospy.spin()

    # ROSInterruptException is normally raised when the node is
    # interrupted/shut down with Ctrl+C or by ROS itself.
    except rospy.ROSInterruptException:
        pass