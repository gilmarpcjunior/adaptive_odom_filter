#!/usr/bin/env python3
# file: multi_fake_odom_imu_rates.py
import threading
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Quaternion

def diag6(vx, vy, vz, vroll, vpitch, vyaw):
    cov = [0.0]*36
    cov[0]  = vx     # x
    cov[7]  = vy     # y
    cov[14] = vz     # z
    cov[21] = vroll  # roll
    cov[28] = vpitch # pitch
    cov[35] = vyaw   # yaw
    return cov

def diag3(a, b, c):
    cov = [0.0]*9
    cov[0] = a
    cov[4] = b
    cov[8] = c
    return cov

class MultiFakeSources:
    def __init__(self):
        # ---------- Params ----------
        # Path/state evolution (1 step per second)
        self.num_points  = rospy.get_param("~num_points", 60)
        self.dx_per_step = rospy.get_param("~dx", 1.0)          # +1 m each second
        self.start_xyz   = rospy.get_param("~start", [0.0, 0.0, 0.0])
        self.update_hz   = rospy.get_param("~state_update_hz", 1.0)  # leave at 1.0 to match +1 m/s

        # Frames & topics
        self.odom_frame  = rospy.get_param("~odom_frame", "odom")
        self.base_frame  = rospy.get_param("~base_frame", "base_link")
        self.imu_frame   = rospy.get_param("~imu_frame", "imu_link")

        self.cam_topic   = rospy.get_param("~camera_odom_topic", "/camera/odom")  # visual odom
        self.wheel_topic = rospy.get_param("~wheel_odom_topic",  "/wheel/odom")
        self.lidar_topic = rospy.get_param("~lidar_odom_topic",  "/lidar/odom")
        self.imu_topic   = rospy.get_param("~imu_topic", "/imu/data")

        # Requested publication rates
        self.cam_rate_hz   = rospy.get_param("~camera_rate_hz", 10.0)   # visual
        self.wheel_rate_hz = rospy.get_param("~wheel_rate_hz",  20.0)
        self.lidar_rate_hz = rospy.get_param("~lidar_rate_hz",  10.0)
        self.imu_rate_hz   = rospy.get_param("~imu_rate_hz",   200.0)

        # Covariances
        pose_var_xyz   = rospy.get_param("~pose_var_xyz", 0.01)  # m^2
        pose_var_rpy   = rospy.get_param("~pose_var_rpy", 0.10)  # rad^2
        twist_var_xyz  = rospy.get_param("~twist_var_xyz", 0.10) # (m/s)^2
        twist_var_rpy  = rospy.get_param("~twist_var_rpy", 0.20) # (rad/s)^2

        self.pose_cov  = diag6(pose_var_xyz, pose_var_xyz, pose_var_xyz,
                               pose_var_rpy, pose_var_rpy, pose_var_rpy)
        self.twist_cov = diag6(twist_var_xyz, twist_var_xyz, twist_var_xyz,
                               twist_var_rpy, twist_var_rpy, twist_var_rpy)

        self.imu_orient_var = rospy.get_param("~imu_var_orient", 0.05)
        self.imu_gyro_var   = rospy.get_param("~imu_var_gyro",   0.01)
        self.imu_accel_var  = rospy.get_param("~imu_var_accel",  0.10)

        # Build guide points (+dx per second)
        self.points = [Point(self.start_xyz[0] + i*self.dx_per_step,
                             self.start_xyz[1], self.start_xyz[2])
                       for i in range(self.num_points)]
        self.i   = 0
        self.lock = threading.Lock()

        # Constant orientation and nominal velocity (matching +dx/s)
        self.q_identity = Quaternion(0.0, 0.0, 0.0, 1.0)
        self.v = self.dx_per_step * self.update_hz  # with 1 Hz & dx=1 → 1 m/s

        # Publishers
        self.cam_pub   = rospy.Publisher(self.cam_topic,   Odometry, queue_size=1)
        self.wheel_pub = rospy.Publisher(self.wheel_topic, Odometry, queue_size=1)
        self.lidar_pub = rospy.Publisher(self.lidar_topic, Odometry, queue_size=1)
        self.imu_pub   = rospy.Publisher(self.imu_topic,   Imu,      queue_size=10)

        # Timers: one that advances state at 1 Hz; others just publish at their own rates
        rospy.Timer(rospy.Duration(1.0/self.update_hz), self._tick_state)
        rospy.Timer(rospy.Duration(1.0/self.cam_rate_hz),   self._pub_camera)
        rospy.Timer(rospy.Duration(1.0/self.wheel_rate_hz), self._pub_wheel)
        rospy.Timer(rospy.Duration(1.0/self.lidar_rate_hz), self._pub_lidar)
        rospy.Timer(rospy.Duration(1.0/self.imu_rate_hz),   self._pub_imu)

    # ---- State evolution: +1 index per second ----
    def _tick_state(self, event):
        with self.lock:
            self.i = (self.i + 1) % len(self.points)

    # ---- Compose Odometry/IMU messages from current state ----
    def _make_odom(self):
        with self.lock:
            pt = self.points[self.i]
        now = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame
        odom.pose.pose.position    = pt
        odom.pose.pose.orientation = self.q_identity
        odom.pose.covariance       = self.pose_cov
        odom.twist.twist.linear.x  = self.v
        odom.twist.twist.linear.y  = 0.0
        odom.twist.twist.linear.z  = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0
        odom.twist.covariance      = self.twist_cov
        return odom

    def _make_imu(self):
        now = rospy.Time.now()
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self.imu_frame
        imu.orientation = self.q_identity
        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = 0.0
        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 0.0
        imu.orientation_covariance         = diag3(self.imu_orient_var,
                                                   self.imu_orient_var,
                                                   self.imu_orient_var)
        imu.angular_velocity_covariance    = diag3(self.imu_gyro_var,
                                                   self.imu_gyro_var,
                                                   self.imu_gyro_var)
        imu.linear_acceleration_covariance = diag3(self.imu_accel_var,
                                                   self.imu_accel_var,
                                                   self.imu_accel_var)
        return imu

    # ---- Publisher callbacks at requested rates ----
    def _pub_camera(self, event):
        self.cam_pub.publish(self._make_odom())

    def _pub_wheel(self, event):
        self.wheel_pub.publish(self._make_odom())

    def _pub_lidar(self, event):
        self.lidar_pub.publish(self._make_odom())

    def _pub_imu(self, event):
        self.imu_pub.publish(self._make_imu())

def main():
    rospy.init_node("multi_fake_odom_imu_rates")
    MultiFakeSources()
    rospy.loginfo("multi_fake_odom_imu_rates started: cam 10 Hz, wheel 20 Hz, lidar 10 Hz, imu 200 Hz")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
