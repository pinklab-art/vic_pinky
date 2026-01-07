#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import time

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

# Import the driver module
from vicpinky_bringup.zlac_driver import ZLACDriver

# --- Configuration Constants ---
TWIST_SUB_TOPIC_NAME = "cmd_vel"
ODOM_PUB_TOPIC_NAME = "odom"
JOINT_PUB_TOPIC_NAME = "joint_states"
ODOM_FRAME_ID = "odom"
ODOM_CHILD_FRAME_ID = "base_footprint"

SERIAL_PORT_NAME = "/dev/motor"
BAUDRATE = 115200
MODBUS_ID = 0x01

JOINT_NAME_WHEEL_L = "left_wheel_joint"
JOINT_NAME_WHEEL_R = "right_wheel_joint"

WHEEL_RAD = 0.0825
PULSE_PER_ROT = 4096
WHEEL_BASE = 0.475
RPM2RAD = 0.104719755
CIRCUMFERENCE = 2 * math.pi * WHEEL_RAD


class VicPinky(Node):
    def __init__(self):
        super().__init__('vic_pinky_bringup')
        self.is_initialized = False

        self.get_logger().info('Initializing Vic Pinky Bringup Node...')
        
        # --- Parameters for Acceleration/Deceleration ---
        self.declare_parameter('accel_limit', 0.4)    
        self.declare_parameter('decel_limit', 1.0)
        
        self.declare_parameter('ang_accel_limit', 1.0)
        self.declare_parameter('ang_decel_limit', 1.5)

        self.accel = self.get_parameter('accel_limit').value
        self.decel = self.get_parameter('decel_limit').value
        
        self.ang_accel = self.get_parameter('ang_accel_limit').value
        self.ang_decel = self.get_parameter('ang_decel_limit').value

        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        
        self.driver = ZLACDriver(SERIAL_PORT_NAME, BAUDRATE, MODBUS_ID)
        
        self.get_logger().info("1. Opening serial port...")
        if not self.driver.begin():
            self.get_logger().error("Failed to open serial port! Shutting down.")
            return
        time.sleep(0.1)

        self.get_logger().info("2. Setting velocity mode...")
        if not self.driver.set_vel_mode():
            self.get_logger().error("Failed to set velocity mode! Shutting down.")
            self.driver.terminate()
            return
        time.sleep(0.1)

        self.get_logger().info("3. Enabling motors...")
        if not self.driver.enable():
            self.get_logger().error("Failed to enable motors! Shutting down.")
            self.driver.terminate()
            return
        
        self.get_logger().info("Waiting for motor controller to be ready...")
        time.sleep(1.0)

        self.get_logger().info("4. Verifying motor controller is responsive...")
        rpm_l, rpm_r = self.driver.get_rpm()
        if rpm_l is None:
            self.get_logger().error("Motor controller is not responding to status requests! Shutting down.")
            self.driver.terminate()
            return
        self.get_logger().info(f"Initial RPM read: L={rpm_l}, R={rpm_r}. Controller is responsive.")
        time.sleep(0.1)

        self.get_logger().info("5. Setting initial RPM to zero (with retries)...")
        max_retries = 3
        success = False
        for i in range(max_retries):
            self.get_logger().info(f"Attempt {i + 1}/{max_retries}...")
            if self.driver.set_double_rpm(0, 0):
                success = True
                break
            self.get_logger().warn(f"Attempt {i + 1} failed. Retrying in 0.2 seconds...")
            time.sleep(0.2)
        
        if not success:
            self.get_logger().error("Failed to set initial RPM after multiple retries! Shutting down.")
            self.driver.terminate()
            return
        
        self.get_logger().info("6. Reading initial encoder values...")
        self.last_encoder_l, self.last_encoder_r = self.driver.get_position()
        if self.last_encoder_l is None or self.last_encoder_r is None:
            self.get_logger().error("Failed to read initial encoder position! Shutting down.")
            self.driver.terminate()
            return
            
        self.odom_pub = self.create_publisher(Odometry, ODOM_PUB_TOPIC_NAME, 10)
        self.joint_pub = self.create_publisher(JointState, JOINT_PUB_TOPIC_NAME, 10)
        self.twist_sub = self.create_subscription(Twist, TWIST_SUB_TOPIC_NAME, self.twist_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / 30.0, self.update_and_publish)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        self.is_initialized = True
        self.get_logger().info('Vic Pinky Bringup has been started successfully.')

    def twist_callback(self, msg: Twist):
        self.target_linear_x = msg.linear.x
        self.target_angular_z = msg.angular.z

    def update_and_publish(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return

        rpm_l_fb, rpm_r_fb = self.driver.get_rpm()
        encoder_l, encoder_r = self.driver.get_position()

        if rpm_l_fb is None or encoder_l is None:
            self.last_time = current_time
            return

        delta_l = encoder_l - self.last_encoder_l
        delta_r = encoder_r - self.last_encoder_r
        
        self.last_encoder_l = encoder_l
        self.last_encoder_r = encoder_r

        dist_l = (delta_l / PULSE_PER_ROT) * CIRCUMFERENCE
        dist_r = (delta_r / PULSE_PER_ROT) * CIRCUMFERENCE

        delta_dist = (dist_r + dist_l) / 2.0
        delta_th = (dist_r - dist_l) / WHEEL_BASE

        self.theta += delta_th
        self.x += delta_dist * math.cos(self.theta)
        self.y += delta_dist * math.sin(self.theta)
        
        v_x = delta_dist / dt
        vth = delta_th / dt

        accel = self.get_parameter('accel_limit').value
        decel = self.get_parameter('decel_limit').value
        ang_accel = self.get_parameter('ang_accel_limit').value
        ang_decel = self.get_parameter('ang_decel_limit').value

        if abs(self.target_linear_x) < abs(v_x):
            step = decel * dt
        elif (self.target_linear_x * v_x) < 0:
            step = decel * dt
        else:
            step = accel * dt

        if v_x < self.target_linear_x:
            self.cmd_linear_x = min(self.target_linear_x, v_x + step)
        elif v_x > self.target_linear_x:
            self.cmd_linear_x = max(self.target_linear_x, v_x - step)
        else:
            self.cmd_linear_x = self.target_linear_x

        if abs(self.target_angular_z) < abs(vth):
            step = ang_decel * dt
        elif (self.target_angular_z * vth) < 0:
            step = ang_decel * dt
        else:
            step = ang_accel * dt

        if vth < self.target_angular_z:
            self.cmd_angular_z = min(self.target_angular_z, vth + step)
        elif vth > self.target_angular_z:
            self.cmd_angular_z = max(self.target_angular_z, vth - step)
        else:
            self.cmd_angular_z = self.target_angular_z

        try:
            v_l = self.cmd_linear_x - (self.cmd_angular_z * WHEEL_BASE / 2.0)
            v_r = self.cmd_linear_x + (self.cmd_angular_z * WHEEL_BASE / 2.0)
            
            rpm_l_raw = v_l / (WHEEL_RAD * RPM2RAD)
            rpm_r_raw = v_r / (WHEEL_RAD * RPM2RAD)
            
            max_rpm_limit = 28.0
            max_req = max(abs(rpm_l_raw), abs(rpm_r_raw))
            
            if max_req > max_rpm_limit:
                scale = max_rpm_limit / max_req
                rpm_l_raw *= scale
                rpm_r_raw *= scale

            rpm_l = int(rpm_l_raw)
            rpm_r = int(rpm_r_raw)
            
            rpm_l = max(min(rpm_l, max_rpm_limit), -max_rpm_limit)
            rpm_r = max(min(rpm_r, max_rpm_limit), -max_rpm_limit)

            self.driver.set_double_rpm(rpm_l, rpm_r)
        except:
            pass

        self._publish_tf(current_time)
        self._publish_odometry(current_time, v_x, vth)
        self._publish_joint_states(current_time, rpm_l_fb * RPM2RAD, rpm_r_fb * RPM2RAD)

        self.last_time = current_time

    def _publish_tf(self, current_time):
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = ODOM_FRAME_ID
        t.child_frame_id = ODOM_CHILD_FRAME_ID
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def _publish_odometry(self, current_time, v_x, vth):
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = ODOM_FRAME_ID
        odom_msg.child_frame_id = ODOM_CHILD_FRAME_ID
        
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        
        q = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        odom_msg.twist.twist.linear.x = v_x
        odom_msg.twist.twist.angular.z = vth
        
        odom_msg.pose.covariance = [0.1] * 36
        odom_msg.twist.covariance = [0.1] * 36
        self.odom_pub.publish(odom_msg)

    def _publish_joint_states(self, current_time, vel_l_rads, vel_r_rads):
        joint_msg = JointState()
        joint_msg.header.stamp = current_time.to_msg()
        joint_msg.name = [JOINT_NAME_WHEEL_L, JOINT_NAME_WHEEL_R]
        joint_msg.position = [
            (self.last_encoder_l / PULSE_PER_ROT) * (2 * math.pi),
            (self.last_encoder_r / PULSE_PER_ROT) * (2 * math.pi)
        ]
        joint_msg.velocity = [vel_l_rads, vel_r_rads]
        self.joint_pub.publish(joint_msg)

    def on_shutdown(self):
        self.get_logger().info("Shutting down, terminating motor driver...")
        if hasattr(self, 'driver') and self.driver:
            self.driver.set_double_rpm(0, 0)
            self.driver.terminate()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VicPinky()
        if hasattr(node, 'is_initialized') and node.is_initialized:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            if hasattr(node, 'is_initialized') and node.is_initialized:
                node.on_shutdown()
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()