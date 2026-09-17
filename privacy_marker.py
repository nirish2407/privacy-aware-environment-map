#!/usr/bin/env python3

import rospy

from std_msgs.msg import String
from visualization_msgs.msg import Marker


class PrivacyMarker:

    def __init__(self):

        rospy.init_node("privacy_marker")

        self.camera_id = rospy.get_param(
            "~camera_id",
            "left"
        )

        self.pub = rospy.Publisher(
            f"/privacy_markers_{self.camera_id}",
            Marker,
            queue_size=10
        )

        rospy.Subscriber(
            f"/privacy_objects_map_{self.camera_id}",
            String,
            self.callback,
            queue_size=10
        )

        self.marker_ids = {}
        self.next_id = 0

        rospy.loginfo(
            "Privacy marker started"
        )

    def get_marker_id(self, name):
        """
        Each unique object key (e.g. "face_3", already including
        the track id from object_position.py) gets one stable
        marker id for its whole lifetime.
        """

        if name not in self.marker_ids:

            self.marker_ids[name] = self.next_id

            self.next_id += 1

        return self.marker_ids[name]

    def callback(self, msg):

        data = msg.data.split(",")

        if len(data) < 2:
            rospy.logwarn(
                "Invalid map object: %s",
                msg.data
            )
            return

        name = data[0]

        # -----------------------------------------------------------
        # FIX: object_position.py can now ask for a marker to be
        # explicitly removed ("<name>,DELETE") when a track goes
        # stale, instead of relying on the marker expiring on its
        # own. This mirrors the reliable stale-track handling that
        # already existed in object_position.py.
        # -----------------------------------------------------------
        if len(data) == 2 and data[1] == "DELETE":
            self.delete_marker(name)
            return

        if len(data) < 4:
            rospy.logwarn(
                "Invalid map object: %s",
                msg.data
            )
            return

        try:
            x = float(data[1])
            y = float(data[2])

            # NOTE: object_position.py now derives z from a real
            # depth-camera measurement (back-projected pixel depth,
            # transformed into the map frame) rather than always
            # sending a fixed placeholder height. No change needed
            # here -- this node already just places the marker at
            # whatever (x, y, z) it's given.
            z = float(data[3])

        except ValueError:
            return

        marker_id = self.get_marker_id(name)

        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()

        marker.ns = "privacy_objects"

        marker.id = marker_id

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # -----------------------------------------------------------
        # FIX: 0.05s meant the marker vanished almost immediately
        # unless a fresh position arrived faster than 20 Hz, causing
        # constant flicker. Markers are now persistent (lifetime=0)
        # and are removed explicitly via the DELETE message above
        # when object_position.py drops the track -- this is a more
        # reliable "stale marker" mechanism than a very short timer.
        # -----------------------------------------------------------
        marker.lifetime = rospy.Duration(0)

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # -----------------------------------------------------------
        # FIX: 0.005 m (5 mm) spheres are effectively invisible at
        # normal RViz zoom levels. Using a size that's actually
        # visible in the map view.
        # -----------------------------------------------------------
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25

        if self.camera_id == "left":
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

        elif self.camera_id == "right":
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0

        marker.color.a = 1.0

        # NOTE: Marker.text is only rendered for TEXT_VIEW_FACING
        # markers, not SPHERE markers, so it has no visible effect
        # here. Left in for anyone inspecting the message on the
        # topic; add a second TEXT_VIEW_FACING marker if an on-screen
        # label is wanted.
        #marker.text = name

        self.pub.publish(marker)

        # -----------------------------------------------------------
        # Text label above the marker
        # -----------------------------------------------------------
        text_marker = Marker()

        text_marker.header.frame_id = "map"
        text_marker.header.stamp = rospy.Time.now()

        text_marker.ns = "privacy_labels"

        # Use a different ID namespace, so it doesn't conflict
        # with the sphere marker.
        text_marker.id = marker_id

        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD

        text_marker.pose.position.x = x
        text_marker.pose.position.y = y
        text_marker.pose.position.z = z + 0.25

        text_marker.pose.orientation.x = 0.0
        text_marker.pose.orientation.y = 0.0
        text_marker.pose.orientation.z = 0.0
        text_marker.pose.orientation.w = 1.0

        # Height of the text in meters
        text_marker.scale.z = 0.12

        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0

        text_marker.text = "Privacy Sensitive Object: " + name.split("_")[0].capitalize()

        text_marker.lifetime = rospy.Duration(0)

        self.pub.publish(text_marker)

    def delete_marker(self, name):

        if name not in self.marker_ids:
            # Nothing was ever published for this key.
            return

        # Save the ID before deleting the dictionary entry.
        marker_id = self.marker_ids[name]

        # ---------------------------------------------------------
        # Delete sphere marker
        # ---------------------------------------------------------

        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()

        marker.ns = "privacy_objects"
        marker.id = marker_id
        marker.action = Marker.DELETE

        self.pub.publish(marker)

        # ---------------------------------------------------------
        # Delete text label
        # ---------------------------------------------------------

        text_marker = Marker()

        text_marker.header.frame_id = "map"
        text_marker.header.stamp = rospy.Time.now()

        text_marker.ns = "privacy_labels"
        text_marker.id = marker_id
        text_marker.action = Marker.DELETE

        self.pub.publish(text_marker)

        # ---------------------------------------------------------
        # Now remove it from our dictionary
        # ---------------------------------------------------------

        del self.marker_ids[name]

        rospy.loginfo(
            "Deleted marker: %s (id=%d)",
            name,
            marker_id
        )

if __name__ == "__main__":

    try:

        PrivacyMarker()

        rospy.spin()

    except rospy.ROSInterruptException:

        pass