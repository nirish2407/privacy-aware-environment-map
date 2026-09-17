#!/usr/bin/env python3

import rospy
import math
import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge

import tf2_ros


class PrivacyObjectPosition(object):

    def __init__(self):

        rospy.init_node("object_position")

        # Detections published by an older/unpatched privacy_detector.py
        # (8 fields, no camera_id) are assumed to come from this
        # camera_id so single-camera setups keep working unchanged.
        self.camera_id = rospy.get_param(
            "~camera_id",
            "left"
        )

        # =========================================================
        # TOPICS
        # =========================================================

        self.detection_topic = f"/privacy_detections_{self.camera_id}"

        # -----------------------------------------------------------
        # FIX (root-cause rewrite #1): positions used to be found by
        # projecting the camera ray to a bearing on the 2D LiDAR
        # plane and reading whatever range the LiDAR happened to
        # return at that bearing. That's only correct when the
        # object is flush against a wall at LiDAR height -- for
        # anything else (a person standing free, an object on a
        # table/shelf) the beam passes under/over it and hits
        # whatever is actually further along that direction. That's
        # exactly what produced markers landing in open floor space
        # in RViz instead of on the detected object.
        #
        # Replaced with a direct depth-camera lookup: read the
        # metric depth at the detected pixel from a depth image that
        # is pixel-registered to the RGB image the detector ran on,
        # then back-project that single (pixel, depth) pair into a
        # real 3D point with the pinhole camera model. No wall/
        # alignment assumption, no LiDAR plane involved.
        #
        # FIX (root-cause rewrite #2 -- coverage): a single torso
        # camera only covers a slice of the space in front of the
        # robot, so most people passing by were never seen at all,
        # and only one of three occasionally clipped that camera's
        # FOV edge. privacy_detector.py now runs one instance per
        # camera and tags each detection with a camera_id. This node
        # keeps a small config PER camera_id (its own depth topic,
        # CameraInfo, and TF frame) instead of a single global one,
        # so detections from either camera get correctly
        # back-projected using that camera's own depth stream and
        # intrinsics.
        # -----------------------------------------------------------
        self.camera_configs = {
            "left": {
                "depth_topic": rospy.get_param(
                    "~left_depth_topic",
                    "/torso_cam3d_left/depth_registered/image_raw"
                ),
                "camera_info_topic": rospy.get_param(
                    "~left_camera_info_topic",
                    "/torso_cam3d_left/rgb/camera_info"
                ),
                "camera_frame": rospy.get_param(
                    "~left_camera_frame",
                    "torso_cam3d_left_rgb_optical_frame"
                ),
                "camera_info": None,
                "depth_buffer": [],
                "warned_no_camera_info": False
            },
            "right": {
                "depth_topic": rospy.get_param(
                    "~right_depth_topic",
                    "/torso_cam3d_right/depth_registered/image_raw"
                ),
                "camera_info_topic": rospy.get_param(
                    "~right_camera_info_topic",
                    "/torso_cam3d_right/rgb/camera_info"
                ),
                "camera_frame": rospy.get_param(
                    "~right_camera_frame",
                    "torso_cam3d_right_rgb_optical_frame"
                ),
                "camera_info": None,
                "depth_buffer": [],
                "warned_no_camera_info": False
            }
        }

        # -----------------------------------------------------------
        # FIX: this node's job is to turn detections + depth into
        # *positions*, and hand them off to privacy_marker.py, which
        # is the node responsible for building RViz Markers.
        # Publishing a plain String here keeps the intended pipeline:
        #   privacy_detector.py -> /privacy_detections
        #   object_position.py  -> /privacy_objects_map
        #   privacy_marker.py   -> /privacy_markers
        # -----------------------------------------------------------
        self.position_topic = f"/privacy_objects_map_{self.camera_id}"

        # =========================================================
        # FRAMES
        # =========================================================

        self.map_frame = "map"
        self.base_frame = "base_link"

        # =========================================================
        # DEPTH PARAMETERS
        # =========================================================

        self.min_depth = 0.15
        self.max_depth = 12.0

        # Maximum allowed detection/depth-image timestamp difference.
        self.max_depth_time_difference = 0.20

        # Half-size (in pixels) of the patch sampled around the
        # bbox-center pixel; the median of valid depth values in
        # that patch is used. Far more robust than reading exactly
        # one pixel, which can easily land on a noisy/zero value.
        self.depth_patch_radius = 2

        # If that patch has no valid depth at all (e.g. the exact
        # center lands on a dropout/reflective spot), grow the patch
        # up to this radius before giving up on the detection.
        self.max_depth_patch_radius = 7

        # =========================================================
        # RANGE STABILITY
        # =========================================================

        # If a tracked object's depth suddenly jumps by more than
        # this amount, reject the measurement. Depth cameras can
        # briefly return a bad value at object edges / IR dropout;
        # this guards a single bad frame from relocating the marker.
        self.max_range_jump = 1.0

        # =========================================================
        # POSITION SMOOTHING
        # =========================================================

        self.smoothing_alpha = 0.20

        # =========================================================
        # TRACKING
        # =========================================================

        # Maximum image-space movement allowed when matching
        # a new detection to an existing track.
        self.max_pixel_tracking_distance = 100.0

        # Delete tracks after this amount of time.
        self.track_timeout = 1.5

        # =========================================================
        # DATA STORAGE
        # =========================================================

        self.bridge = CvBridge()

        self.max_depth_buffer_size = 40

        #
        # Track structure:
        #
        # tracks[id] = {
        #     "name": "face",
        #     "camera_id": "left",   <- FIX: which camera this track
        #                               belongs to. Matching is scoped
        #                               to (name, camera_id) so pixel
        #                               coordinates from two different
        #                               cameras never get cross-matched
        #                               into the same track.
        #     "u": center x,
        #     "v": center y,
        #     "position": map position (x, y, z),
        #     "range": last measured depth (m),
        #     "last_seen": timestamp
        # }
        #
        # NOTE (known limitation): if a person is visible to BOTH
        # cameras at once (overlap zone), each camera produces its
        # own independent track for them, which can show up as two
        # markers for one physical person. Fusing same-class tracks
        # that land at nearly the same map position across cameras
        # would resolve this if it becomes a problem in practice.
        #
        self.tracks = {}

        self.next_track_id = 0

        # =========================================================
        # TF
        # =========================================================

        self.tf_buffer = tf2_ros.Buffer(
            rospy.Duration(30.0)
        )

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer
        )

        # =========================================================
        # ROS
        # =========================================================

        self.position_pub = rospy.Publisher(
            self.position_topic,
            String,
            queue_size=50
        )

        # -----------------------------------------------------------
        # FIX: one CameraInfo + depth subscriber pair PER camera,
        # each tagged with its camera_id via callback_args so the
        # callbacks know which config dict to update.
        # -----------------------------------------------------------
        for camera_id, config in self.camera_configs.items():

            rospy.Subscriber(
                config["camera_info_topic"],
                CameraInfo,
                self.camera_info_callback,
                callback_args=camera_id,
                queue_size=1
            )

            rospy.Subscriber(
                config["depth_topic"],
                Image,
                self.depth_callback,
                callback_args=camera_id,
                queue_size=10
            )

        rospy.Subscriber(
            self.detection_topic,
            String,
            self.detection_callback,
            queue_size=20
        )

        rospy.loginfo("==============================================")
        rospy.loginfo("Privacy Object Position Node (depth camera)")
        rospy.loginfo("==============================================")

        for camera_id, config in self.camera_configs.items():
            rospy.loginfo(
                "[%s] frame=%s depth=%s info=%s",
                camera_id,
                config["camera_frame"],
                config["depth_topic"],
                config["camera_info_topic"]
            )

        rospy.loginfo("Map:           %s", self.map_frame)
        rospy.loginfo("Output:        %s", self.position_topic)
        rospy.loginfo("Multi-object tracking: ENABLED")
        rospy.loginfo("==============================================")

    # =============================================================
    # CAMERA INFO CALLBACK
    # =============================================================

    def camera_info_callback(self, msg, camera_id):

        self.camera_configs[camera_id]["camera_info"] = msg

    # =============================================================
    # DEPTH CALLBACK
    # =============================================================

    def depth_callback(self, msg, camera_id):

        buffer = self.camera_configs[camera_id]["depth_buffer"]

        buffer.append(msg)

        if len(buffer) > self.max_depth_buffer_size:
            buffer.pop(0)

    # =============================================================
    # DETECTION CALLBACK
    # =============================================================

    def detection_callback(self, msg):

        detections = []

        # ---------------------------------------------------------
        # Parse all detections from this image
        # ---------------------------------------------------------

        for text in msg.data.split("|"):

            text = text.strip()

            if not text:
                continue

            parts = text.split(",")

            if len(parts) < 8:
                rospy.logwarn(
                    "Invalid detection: %s",
                    text
                )
                continue

            try:

                name = parts[0]

                confidence = float(parts[1])

                x1 = float(parts[2])
                y1 = float(parts[3])
                x2 = float(parts[4])
                y2 = float(parts[5])

                secs = int(parts[6])
                nsecs = int(parts[7])

                stamp = rospy.Time(
                    secs,
                    nsecs
                )

                # -----------------------------------------------------
                # FIX: 9th field is the source camera_id, added so
                # detections from multiple cameras can each be
                # back-projected with the right depth/intrinsics/TF
                # frame. Older detectors that only send 8 fields are
                # assumed to be the configured default camera, so a
                # single-camera setup keeps working unchanged.
                # -----------------------------------------------------
                if len(parts) >= 9:
                    camera_id = parts[8]
                else:
                    camera_id = self.default_camera_id

                if camera_id not in self.camera_configs:
                    rospy.logwarn(
                        "Unknown camera_id '%s' in detection, skipping",
                        camera_id
                    )
                    continue

                # BBox center
                u = (x1 + x2) / 2.0
                v = (y1 + y2) / 2.0

                detections.append({
                    "name": name,
                    "camera_id": camera_id,
                    "confidence": confidence,
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "u": u,
                    "v": v,
                    "stamp": stamp
                })

            except Exception as e:

                rospy.logwarn(
                    "Could not parse detection: %s",
                    str(e)
                )

        if len(detections) == 0:
            return

        rospy.loginfo(
            "Detected %d privacy object(s)",
            len(detections)
        )

        # ---------------------------------------------------------
        # Match detections to existing tracks
        # ---------------------------------------------------------

        assignments = self.assign_tracks(
            detections
        )

        # ---------------------------------------------------------
        # Process each object
        # ---------------------------------------------------------

        for detection, track_id in assignments:

            self.process_detection(
                detection,
                track_id
            )

        # ---------------------------------------------------------
        # Remove old tracks
        # ---------------------------------------------------------

        self.remove_stale_tracks()

    # =============================================================
    # MULTI-OBJECT TRACKING
    # =============================================================

    def assign_tracks(self, detections):

        assignments = []

        used_tracks = set()

        # ---------------------------------------------------------
        # Sort detections from top to bottom.
        #
        # This gives deterministic assignment for multiple faces.
        # ---------------------------------------------------------

        detections = sorted(
            detections,
            key=lambda d: d["v"]
        )

        for detection in detections:

            name = detection["name"]
            camera_id = detection["camera_id"]
            u = detection["u"]
            v = detection["v"]

            best_id = None
            best_distance = float("inf")

            # -----------------------------------------------------
            # Find nearest existing track in image space.
            #
            # FIX: also require the same camera_id. Pixel (u, v) is
            # meaningless across two different cameras' images --
            # without this, a detection from the right camera could
            # get matched to a track that was actually created from
            # the left camera just because the numbers were close.
            # -----------------------------------------------------

            for track_id, track in self.tracks.items():

                if track_id in used_tracks:
                    continue

                if track["name"] != name:
                    continue

                if track["camera_id"] != camera_id:
                    continue

                du = u - track["u"]
                dv = v - track["v"]

                pixel_distance = math.sqrt(
                    du * du + dv * dv
                )

                if (
                    pixel_distance
                    <= self.max_pixel_tracking_distance
                    and
                    pixel_distance < best_distance
                ):

                    best_distance = pixel_distance
                    best_id = track_id

            # -----------------------------------------------------
            # Create a new persistent track.
            # -----------------------------------------------------

            if best_id is None:

                best_id = self.next_track_id

                self.next_track_id += 1

                self.tracks[best_id] = {
                    "name": name,
                    "camera_id": camera_id,
                    "u": u,
                    "v": v,
                    "position": None,
                    "range": None,
                    "last_seen": detection["stamp"]
                }

                rospy.loginfo(
                    "NEW TRACK: %s_%d (camera=%s)",
                    name,
                    best_id,
                    camera_id
                )

            # -----------------------------------------------------
            # Update image position.
            # -----------------------------------------------------

            self.tracks[best_id]["u"] = u
            self.tracks[best_id]["v"] = v
            self.tracks[best_id]["last_seen"] = detection["stamp"]

            used_tracks.add(best_id)

            assignments.append(
                (detection, best_id)
            )

        return assignments

    # =============================================================
    # PROCESS ONE OBJECT
    # =============================================================

    def process_detection(
        self,
        detection,
        track_id
    ):

        name = detection["name"]
        camera_id = detection["camera_id"]
        confidence = detection["confidence"]

        u = detection["u"]
        v = detection["v"]

        detection_time = detection["stamp"]

        config = self.camera_configs[camera_id]

        rospy.loginfo("")
        rospy.loginfo("==============================================")

        rospy.loginfo(
            "Object: %s_%d confidence=%.3f camera=%s",
            name,
            track_id,
            confidence,
            camera_id
        )

        rospy.loginfo(
            "BBox: %.0f %.0f %.0f %.0f",
            detection["x1"],
            detection["y1"],
            detection["x2"],
            detection["y2"]
        )

        rospy.loginfo(
            "BBox center: u=%.1f v=%.1f",
            u,
            v
        )

        # =========================================================
        # INTRINSICS AVAILABLE?
        # =========================================================

        if config["camera_info"] is None:

            if not config["warned_no_camera_info"]:

                rospy.logwarn(
                    "No CameraInfo received yet on %s -- "
                    "cannot compute 3D position for camera=%s",
                    config["camera_info_topic"],
                    camera_id
                )

                config["warned_no_camera_info"] = True

            return

        fx = config["camera_info"].K[0]
        fy = config["camera_info"].K[4]
        cx = config["camera_info"].K[2]
        cy = config["camera_info"].K[5]

        # =========================================================
        # CLOSEST DEPTH IMAGE (from this detection's own camera)
        # =========================================================

        depth_msg = self.find_closest_depth(
            config["depth_buffer"],
            detection_time
        )

        if depth_msg is None:

            rospy.logwarn(
                "%s_%d: no depth image",
                name,
                track_id
            )

            return

        time_difference = abs(
            (
                depth_msg.header.stamp
                - detection_time
            ).to_sec()
        )

        rospy.loginfo(
            "Depth timestamp difference: %.3f sec",
            time_difference
        )

        if time_difference > self.max_depth_time_difference:

            rospy.logwarn(
                "%s_%d: depth image too old",
                name,
                track_id
            )

            return

        # =========================================================
        # DEPTH AT PIXEL
        # =========================================================

        depth_value = self.sample_depth(
            depth_msg,
            u,
            v
        )

        if depth_value is None:

            rospy.logwarn(
                "REJECT %s_%d: no valid depth near pixel (%.0f, %.0f)",
                name,
                track_id,
                u,
                v
            )

            return

        rospy.loginfo(
            "Depth at pixel: %.3f m",
            depth_value
        )

        if depth_value < self.min_depth or depth_value > self.max_depth:

            rospy.logwarn(
                "REJECT %s_%d: depth %.3f m out of range",
                name,
                track_id,
                depth_value
            )

            return

        # =========================================================
        # RANGE JUMP CHECK
        # =========================================================

        previous_range = self.tracks[
            track_id
        ]["range"]

        if previous_range is not None:

            range_difference = abs(
                depth_value - previous_range
            )

            rospy.loginfo(
                "Range change: %.3f m",
                range_difference
            )

            if range_difference > self.max_range_jump:

                rospy.logwarn(
                    "REJECT %s_%d: range jumped %.2f -> %.2f m",
                    name,
                    track_id,
                    previous_range,
                    depth_value
                )

                return

        # Save valid range.
        self.tracks[
            track_id
        ]["range"] = depth_value

        # =========================================================
        # PIXEL + DEPTH -> 3D POINT IN CAMERA OPTICAL FRAME
        #
        # Standard pinhole back-projection: given metric depth Z at
        # pixel (u, v) and intrinsics (fx, fy, cx, cy), the point in
        # the camera's optical frame is:
        #   X = (u - cx) * Z / fx
        #   Y = (v - cy) * Z / fy
        #   Z = Z
        # =========================================================

        z = depth_value
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        point_camera = np.array([x, y, z])

        rospy.loginfo(
            "Camera-frame point: %.3f %.3f %.3f",
            point_camera[0],
            point_camera[1],
            point_camera[2]
        )

        # =========================================================
        # CAMERA -> MAP
        #
        # FIX: this is now a real metric point (not just a
        # direction), so the full transform (rotation + translation)
        # is applied directly from the camera optical frame to map --
        # no intermediate LiDAR frame/plane involved.
        # =========================================================

        tf_map_camera = self.lookup_transform(
            self.map_frame,
            config["camera_frame"],
            depth_msg.header.stamp
        )

        if tf_map_camera is None:
            return

        R_map_camera, t_map_camera = tf_map_camera

        measured_map = (
            np.dot(
                R_map_camera,
                point_camera
            )
            + t_map_camera
        )

        rospy.loginfo(
            "Measured map position: %.3f %.3f %.3f",
            measured_map[0],
            measured_map[1],
            measured_map[2]
        )

        # =========================================================
        # MAP POSITION JUMP CHECK
        # =========================================================

        previous_position = self.tracks[
            track_id
        ]["position"]

        if previous_position is not None:

            map_jump = np.linalg.norm(
                measured_map[:2]
                - previous_position[:2]
            )

            rospy.loginfo(
                "Map position change: %.3f m",
                map_jump
            )

            #
            # Do not completely reject large map changes based
            # only on this test, because the robot itself moves.
            #
            # The range-jump test above is the primary protection.
            #

        # =========================================================
        # SMOOTH POSITION
        # =========================================================

        if previous_position is None:

            filtered_map = measured_map.copy()

        else:

            alpha = self.smoothing_alpha

            filtered_map = (
                alpha * measured_map
                +
                (1.0 - alpha) * previous_position
            )

        self.tracks[
            track_id
        ]["position"] = filtered_map

        rospy.loginfo(
            "Filtered map position: %.3f %.3f %.3f",
            filtered_map[0],
            filtered_map[1],
            filtered_map[2]
        )

        # =========================================================
        # ROBOT POSITION IN MAP
        # =========================================================

        robot_tf = self.lookup_transform(
            self.map_frame,
            self.base_frame,
            depth_msg.header.stamp
        )

        if robot_tf is None:
            return

        _, robot_position = robot_tf

        # =========================================================
        # ROBOT -> OBJECT DISTANCE
        # =========================================================

        dx = (
            filtered_map[0]
            - robot_position[0]
        )

        dy = (
            filtered_map[1]
            - robot_position[1]
        )

        distance_xy = math.sqrt(
            dx * dx + dy * dy
        )

        rospy.loginfo(
            "Robot map position: %.3f %.3f %.3f",
            robot_position[0],
            robot_position[1],
            robot_position[2]
        )

        rospy.loginfo(
            "Robot -> %s_%d distance: %.3f m",
            name,
            track_id,
            distance_xy
        )

        # =========================================================
        # PUBLISH POSITION
        #
        # FIX: publish a String on /privacy_objects_map instead of
        # building a Marker here. privacy_marker.py owns turning
        # this into an RViz Marker. The track id is embedded in the
        # name field ("name_trackid") so that multiple instances of
        # the same class (e.g. two faces) get distinct markers
        # downstream instead of overwriting each other.
        # =========================================================

        self.publish_position(
            track_id,
            name,
            filtered_map
        )

    # =============================================================
    # FIND CLOSEST DEPTH IMAGE
    # =============================================================

    def find_closest_depth(self, depth_buffer, timestamp):
        """
        depth_buffer is the per-camera buffer for whichever camera
        produced the detection being processed (see
        self.camera_configs[camera_id]["depth_buffer"]).
        """

        if not depth_buffer:
            return None

        best_depth = None
        best_difference = float("inf")

        for depth_msg in depth_buffer:

            difference = abs(
                (
                    depth_msg.header.stamp
                    - timestamp
                ).to_sec()
            )

            if difference < best_difference:

                best_difference = difference
                best_depth = depth_msg

        return best_depth

    # =============================================================
    # SAMPLE DEPTH AT PIXEL
    # =============================================================

    def sample_depth(self, depth_msg, u, v):
        """
        Read the metric depth (in metres) near pixel (u, v) in the
        given depth image.

        A small patch around the pixel is sampled (rather than just
        the single pixel) and the median of the valid values is
        returned -- this is robust to a single noisy/zero-depth
        pixel landing exactly on the bbox center. If the initial
        patch has no valid values at all (e.g. it lands on a depth
        dropout), the patch is grown up to max_depth_patch_radius
        before giving up.

        Handles both common depth encodings:
          - 16UC1: integer depth in millimetres
          - 32FC1: float depth in metres
        """

        try:
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding="passthrough"
            )
        except Exception as e:
            rospy.logwarn(
                "cv_bridge depth conversion failed: %s",
                str(e)
            )
            return None

        height, width = depth_image.shape[:2]

        center_u = int(round(u))
        center_v = int(round(v))

        if not (0 <= center_u < width and 0 <= center_v < height):
            return None

        if depth_image.dtype == np.uint16:
            # 16UC1 depth images are conventionally in millimetres.
            scale = 1.0 / 1000.0
        else:
            # 32FC1 (and anything else) assumed already in metres.
            scale = 1.0

        radius = self.depth_patch_radius

        while radius <= self.max_depth_patch_radius:

            u_min = max(0, center_u - radius)
            u_max = min(width, center_u + radius + 1)
            v_min = max(0, center_v - radius)
            v_max = min(height, center_v + radius + 1)

            patch = (
                depth_image[v_min:v_max, u_min:u_max]
                .astype(np.float32)
                * scale
            )

            valid = patch[
                np.isfinite(patch)
                &
                (patch > 0.0)
            ]

            if valid.size > 0:
                return float(np.median(valid))

            radius += 2

        return None

    # =============================================================
    # TF LOOKUP
    # =============================================================

    def lookup_transform(
        self,
        target_frame,
        source_frame,
        timestamp
    ):

        try:

            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                timestamp,
                rospy.Duration(0.5)
            )

        except Exception as e:

            rospy.logwarn(
                "TF failed: %s <- %s : %s",
                target_frame,
                source_frame,
                str(e)
            )

            return None

        q = transform.transform.rotation

        x = q.x
        y = q.y
        z = q.z
        w = q.w

        # ---------------------------------------------------------
        # Quaternion -> rotation matrix
        # ---------------------------------------------------------

        R = np.array([

            [
                1.0 - 2.0 * (y*y + z*z),
                2.0 * (x*y - z*w),
                2.0 * (x*z + y*w)
            ],

            [
                2.0 * (x*y + z*w),
                1.0 - 2.0 * (x*x + z*z),
                2.0 * (y*z - x*w)
            ],

            [
                2.0 * (x*z - y*w),
                2.0 * (y*z + x*w),
                1.0 - 2.0 * (x*x + y*y)
            ]

        ])

        t = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])

        return R, t

    # =============================================================
    # PUBLISH POSITION
    # =============================================================

    def publish_position(
        self,
        track_id,
        name,
        position
    ):
        """
        Publish a comma-separated position update for this track on
        /privacy_objects_map, in the format expected by
        privacy_marker.py:

            "<name>_<track_id>,<x>,<y>,<z>"

        Embedding the track id in the name field guarantees a unique
        key per physical object, so privacy_marker.py can show
        several simultaneous instances of the same class without
        them overwriting each other's marker.

        FIX: z is now the real measured height from the depth
        camera, not a fixed placeholder (the old 2D-LiDAR pipeline
        couldn't measure height at all and always published a
        hard-coded z=1.2).
        """

        unique_name = f"{name}_{track_id}"

        data = (
            f"{unique_name},"
            f"{float(position[0]):.3f},"
            f"{float(position[1]):.3f},"
            f"{float(position[2]):.3f}"
        )

        self.position_pub.publish(data)

        rospy.loginfo(
            "Published %s -> map [%.3f %.3f %.3f]",
            unique_name,
            position[0],
            position[1],
            position[2]
        )

    # =============================================================
    # REMOVE STALE TRACKS
    # =============================================================

    def remove_stale_tracks(self):

        now = rospy.Time.now()

        stale_tracks = []

        for track_id, track in self.tracks.items():

            age = (
                now
                - track["last_seen"]
            ).to_sec()

            if age > self.track_timeout:

                stale_tracks.append(
                    track_id
                )

        for track_id in stale_tracks:

            name = self.tracks[
                track_id
            ]["name"]

            rospy.loginfo(
                "Removing stale %s_%d",
                name,
                track_id
            )

            self.publish_deletion(
                track_id,
                name
            )

            del self.tracks[
                track_id
            ]

    # =============================================================
    # PUBLISH DELETION
    # =============================================================

    def publish_deletion(self, track_id, name):
        """
        Tell privacy_marker.py to remove the marker for this track,
        using the same "<name>_<track_id>" key used when publishing
        positions.
        """

        unique_name = f"{name}_{track_id}"

        data = f"{unique_name},DELETE"

        self.position_pub.publish(data)


# =================================================================
# MAIN
# =================================================================

if __name__ == "__main__":

    try:

        node = PrivacyObjectPosition()

        rospy.spin()

    except rospy.ROSInterruptException:

        pass