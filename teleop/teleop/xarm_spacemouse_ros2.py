import rclpy
from rclpy.node import Node

import numpy as np
import tkinter as tk
from xarm.wrapper import XArmAPI
import time  # For measuring press durations

from sensor_msgs.msg import Joy #added this
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import TwistStamped, PoseStamped
#from scipy.spatial.transform import Rotation

from pathlib import Path

START_FLAG = Path("/tmp/start_demo")
STOP_FLAG  = Path("/tmp/stop_demo")

#def euler_to_quaternion(roll, pitch, yaw):
   # Assuming degrees for the xArm, as in your original code
   #rot = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True)
   #quat = rot.as_quat()
   # print(quat)
   #return quat


class Spacemouse2Xarm(Node):
    def __init__(self):
        super().__init__('spacemouse2xarm')
        ip = self.declare_parameter('xarm_ip', '192.168.1.219').value


        # --- XArm setup ---
        self.arm = XArmAPI(ip)
        if self.arm.get_state() != 0:
            self.arm.clean_error()
            time.sleep(0.5)
        self.arm.motion_enable(enable=True)
        self.arm.set_gripper_enable(enable=True) 
        self.arm.set_gripper_mode(0)
        self.arm.set_mode(1)
        self.arm.set_state(0)
        time.sleep(2.0)

        self.latest_msg = None
        self.latest_axes = np.zeros(6)  # [vx, vy, vz, wx, wy, wz]
        self.gripper_position = 0.0
        self.left_button = False
        self.right_button = False
        # Track previous button states for edge detection
        self.prev_left_button = False
        self.prev_right_button = False


        #added this, Joystick subscription
        self.joy_sub = self.create_subscription(Joy, 'spacenav/joy', self.joystick_callback, 10)


        # Additional state for tracking long-press on right button
        self.right_button_pressed = False
        self.right_button_press_time = None
        self.go_home_done_for_press = False
        self.long_press_threshold = 1.0  # seconds (adjust as needed)


        # --- Create publishers ---
        self.action_pub = self.create_publisher(TwistStamped, 'robot_action', 10)
        self.position_action_pub = self.create_publisher(PoseStamped, 'robot_position_action', 10)
        self.gripper_pub = self.create_publisher(Float32, 'gripper_position', 10)

        # --- Listen for external reset messages ---
        self.reset_sub = self.create_subscription(Bool, 'reset_xarm', self.reset_callback, 10)
        self.is_resetting = False


        # --- GUI for the gripper slider (UPDATED with larger size) ---
        self.root = tk.Tk()
        self.root.title("Gripper Control")
        self.root.geometry("1500x300")  # Larger window
        
        self.slider = tk.Scale(
            self.root,
            from_=0, to=1,
            resolution=0.01,
            orient='horizontal',
            label='Gripper Open/Close',
            command=self.update_gripper,
            length=1700,  # Longer slider
            width=100,     # Thicker slider
            font=('Arial', 16, 'bold'),  # Larger font
            troughcolor='#E0E0E0',  # Light gray background
            sliderlength=80  # Larger slider handle
        )
        self.slider.pack(fill=tk.X, expand=True, padx=20, pady=50)  # More padding
        self.slider.set(0)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.gripper_position = 0.0
        self.root.update()

        self.dt = 1.0/30.0
        # --- Timer for ~60 Hz loop ---
        self.timer = self.create_timer(self.dt, self.timer_callback)


    def update_gripper(self, value):
        self.gripper_position = float(value)


    def reset_callback(self, msg: Bool):
        """
        Called externally via the "reset_xarm" topic.
        """
        # let's try to modify the reset position and focus only on grasping the object.
        if msg.data:
            self.is_resetting = True
            self.arm.motion_enable(enable=True)
            self.arm.set_mode(0)
            self.arm.set_state(0)
            self.arm.set_position(
                x=166.9, y=2.1, z=230.5,
                roll=179.2, pitch=0.1, yaw=1.3,
                speed=100, is_radian=False, wait=True
            )
            self.arm.motion_enable(enable=True)
            self.arm.set_mode(1)
            self.arm.set_state(0)
            self.is_resetting = False


    def joystick_callback(self, msg: Joy):
        """
        Called by timer_callback
        """
        self.latest_axes = np.array(msg.axes[:6])  # [x, y, z, roll, pitch, yaw]
        self.latest_msg = msg

    def go_home(self):
        """
        Same motion logic as in reset_callback, to move the arm home.
        """
        self.is_resetting = True
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0) # Position control mode
        self.arm.set_state(0)
        self.arm.set_position(
                x=187.4, y=2.9, z=405.8,
                roll=-179.4, pitch=50.2, yaw=179.9,
                speed=100, is_radian=False, wait=True
            )
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0) 
        self.arm.set_state(0)
        self.is_resetting = False

    def go_home_joints(self):
        """
        Same motion logic as in reset_callback, to move the arm home.
        """
        self.is_resetting = True
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_gripper_enable(enable=True)
        self.arm.set_gripper_mode(0)
        self.arm.set_state(0)
        cmd_joint_pose = [0.0, -81.3, -26.5, 0.0, 58.0, 180.0] 
        cmd_gripper_pose = 850.0
        print("going home")
        self.arm.set_servo_angle(servo_id=8, angle=cmd_joint_pose, is_radian=False, wait=True) 
        self.arm.set_gripper_position(cmd_gripper_pose, wait=True)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(1) # Servo control mode
        self.arm.set_state(0)
        self.is_resetting = False

    def timer_callback(self):
        """
        Main 60 Hz update: reads SpaceMouse, publishes robot control messages,
        optionally commands xArm, etc.
        """
       
        if self.latest_msg is None:
            return  # If no joystick message has been received yet, skip this update

        # Now, you can safely reference `self.latest_msg`
        left_button = bool(self.latest_msg.buttons[0])
        right_button = bool(self.latest_msg.buttons[1])


        # =============================
        #   BUTTON EDGE DETECTIONS
        # =============================
        # Left Button => "start_demo" on rising edge
        if left_button and not self.prev_left_button:
            START_FLAG.touch()
            print("left button pressed, starting demo")


        # Right Button => "end_demo" on rising edge
        if right_button and not self.prev_right_button:
            STOP_FLAG.touch()
            print("right button pressed, ending demo")
            # Start tracking how long it's held
            self.right_button_pressed = True
            self.right_button_press_time = time.time()
            self.go_home_done_for_press = False


        # Right Button => falling edge => reset the press flags
        if not right_button and self.prev_right_button:
            self.right_button_pressed = False
            self.right_button_press_time = None
            self.go_home_done_for_press = False


        # ===========================
        #   LONG-PRESS DETECTION
        # ===========================
        if self.right_button_pressed and right_button:
            # It's still held; check duration
            press_duration = time.time() - self.right_button_press_time
            # If we cross the threshold and haven't gone home yet, do it
            if (press_duration >= self.long_press_threshold) and (not self.go_home_done_for_press):
                self.go_home_joints()
                self.go_home_done_for_press = True


        # ======================
        #  Robot Control Loop
        # ======================
        # Skip normal cartesian control if we are actively resetting
        if self.is_resetting:
            self.root.update()
            self.prev_left_button  = left_button
            self.prev_right_button = right_button
            return


        # 1) Get current xArm pose
        curr_pose = self.arm.get_position()[1]
        curr_pose = np.array(curr_pose)

        # 2) Get cartesian input from the SpaceMouse
        scale_linear = 140.0
        scale_angular = 20.0
        vx, vy, vz, wx, wy, wz = self.latest_axes * np.array([scale_linear]*3 + [scale_angular]*3)

        # 3) Compute new pose
        new_pose = curr_pose + (np.array([vx, vy, vz, -wx, -wy, wz]) * self.dt)


        # 4) Publish PoseStamped, note these are not what actually commands the robot
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = new_pose[0]
        pose_msg.pose.position.y = new_pose[1]
        pose_msg.pose.position.z = new_pose[2]
        #q = euler_to_quaternion(new_pose[3], new_pose[4], new_pose[5])
        #pose_msg.pose.orientation.x = q[0]
        #pose_msg.pose.orientation.y = q[1]
        #pose_msg.pose.orientation.z = q[2]
        #pose_msg.pose.orientation.w = q[3]
        #self.position_action_pub.publish(pose_msg)


        # 5) Publish TwistStamped, note these are not what actually commands the robot
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.twist.linear.x = vx
        twist_msg.twist.linear.y = vy
        twist_msg.twist.linear.z = vz
        twist_msg.twist.angular.x = wx
        twist_msg.twist.angular.y = wy
        twist_msg.twist.angular.z = wz
        self.action_pub.publish(twist_msg)


        # 6) Convert the slider [0..1] to a gripper command (0 => 850, 1 => -10)
        grasp = 850 - 860 * self.gripper_position


        # 7) Publish the gripper [0..1]
        self.gripper_pub.publish(Float32(data=self.gripper_position))

        # 8) Command the xArm
        self.arm.set_servo_cartesian(new_pose, speed=300, mvacc=2000)
        self.arm.set_gripper_position(grasp)

        # Final step: update GUI & remember button states
        self.root.update()
        self.prev_left_button  = left_button
        self.prev_right_button = right_button


    def on_close(self):
        self.arm.disconnect()
        self.root.quit()


def main(args=None):
    rclpy.init(args=args)
    xarm_spacemouse = Spacemouse2Xarm()
    rclpy.spin(xarm_spacemouse)
    xarm_spacemouse.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
