#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle a odom sensor
"""

from carla_ros_bridge.pseudo_actor import PseudoActor

from nav_msgs.msg import Odometry


class OdometrySensor(PseudoActor):

    """
    Pseudo odometry sensor
    """

    def __init__(self, uid, name, parent, node):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param carla_world: carla world object
        :type carla_world: carla.World
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        """

        super(OdometrySensor, self).__init__(uid=uid,
                                             name=name,
                                             parent=parent,
                                             node=node)

        self.odometry_publisher = node.new_publisher(Odometry,
                                                     self.get_topic_prefix(),
                                                     qos_profile=10)

    def destroy(self):
        super(OdometrySensor, self).destroy()
        self.node.destroy_publisher(self.odometry_publisher)

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.odom"

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        """
        odometry = Odometry(header=self.parent.get_msg_header("map", timestamp=timestamp))
        odometry.child_frame_id = self.parent.get_prefix()
        try:
            odometry.pose.pose = self.parent.get_current_ros_pose()
            odometry.twist.twist = self.parent.get_current_ros_twist_rotated()
            roll, pitch, yaw = self.parent.get_current_ros_rpy()
            odometry.pose.covariance[0] = roll
            odometry.pose.covariance[1] = pitch
            odometry.pose.covariance[2] = yaw
            odometry.pose.covariance[3] = 7.77
        except AttributeError:
            # parent actor disappeared, do not send tf
            self.node.logwarn(
                "OdometrySensor could not publish. parent actor {} not found".format(self.parent.uid))
            return
        self.odometry_publisher.publish(odometry)
