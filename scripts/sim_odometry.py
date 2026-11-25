#!/usr/bin/env python3
"""Odométrie simulée « roues codeuses ».

Sur le vrai robot, les roues d'odométrie sont distinctes des roues de propulsion : quand le
robot bute contre un mur, l'odométrie s'arrête même si les roues motrices patinent. L'odom du
DiffDrive Gazebo, elle, intègre la rotation des roues motrices et continue d'avancer contre un
mur. Ce nœud la remplace :

- entrée : pose réelle du châssis (plugin gz OdometryPublisher, repère monde = map) ;
- intègre les déplacements réels (avance + rotation) dans le repère du robot, à partir de 0 ;
- publie le TF odom -> base_link et nav_msgs/Odometry sur `odom_simu`.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class SimOdometry(Node):
    def __init__(self):
        super().__init__('sim_odometry')
        self.prev = None  # (x, y, yaw, t) réel précédent
        self.x = self.y = self.yaw = 0.0  # pose odom intégrée

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom_simu', 10)
        self.create_subscription(Odometry, 'odom_ground_truth', self.on_truth, 50)

    def on_truth(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.prev is None:
            self.prev = (p.x, p.y, yaw, t)
            self.publish(msg.header.stamp, 0.0, 0.0)
            return

        px, py, pyaw, pt = self.prev
        self.prev = (p.x, p.y, yaw, t)

        # Déplacement réel exprimé dans le repère du robot (avance seulement : des roues
        # codeuses ne voient pas le glissement latéral)
        dx, dy = p.x - px, p.y - py
        ds = dx * math.cos(pyaw) + dy * math.sin(pyaw)
        dyaw = wrap(yaw - pyaw)

        mid = self.yaw + dyaw / 2.0
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.yaw = wrap(self.yaw + dyaw)

        dt = t - pt
        v = ds / dt if dt > 0 else 0.0
        w = dyaw / dt if dt > 0 else 0.0
        self.publish(msg.header.stamp, v, w)

    def publish(self, stamp, v, w):
        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = SimOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
