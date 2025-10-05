#!/usr/bin/env python3
"""eCAL to MAVROS Vision Pose Bridge.

This script bridges eCAL odometry messages to ROS/MAVROS vision pose messages.
It subscribes to eCAL odometry topics and publishes to /mavros/vision_pose/pose
for PX4 external position estimation.
"""

import sys
import time
import argparse

import capnp
import numpy as np

import ecal.core.core as ecal_core

import pathlib

current_path = str(pathlib.Path(__file__).parent.resolve())
print("working in path " + current_path)

# Import capnp message definition paths
path_to_add = pathlib.Path(current_path) / '../../vk_sdk/capnp'
sys.path.append(str(path_to_add.resolve()))
sys.path.append("/opt/vilota/messages")

capnp.add_import_hook()
import odometry3d_capnp as eCALOdometry3d

# ROS imports (using PoseStamped for mavros/vision_pose/pose)
import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations


from ecal.core.subscriber import MessageSubscriber

class ByteSubscriber(MessageSubscriber):
    """eCAL raw byte subscriber."""
    def __init__(self, name):
        topic_type = "base:byte"
        super(ByteSubscriber, self).__init__(name, topic_type)
        self.callback = None

    def receive(self, timeout=0):
        ret, msg, time = self.c_subscriber.receive(timeout)
        return ret, msg, time

    def set_callback(self, callback):
        self.callback = callback
        self.c_subscriber.set_callback(self._on_receive)

    def rem_callback(self, callback):
        self.c_subscriber.rem_callback(self._on_receive)
        self.callback = None

    def _on_receive(self, topic_name, msg, time):
        self.callback(topic_name, msg, time)    


class MavrosVisionPosePublisher:
    """Publisher class: receives pose from eCAL and publishes to /mavros/vision_pose/pose."""
    
    def __init__(self, ecal_topic: str, use_monotonic: bool, is_ned_input: bool) -> None:
        """Initialize the MAVROS vision pose publisher.
        
        Args:
            ecal_topic: eCAL topic to subscribe to
            use_monotonic: Whether to use eCAL message timestamps
            is_ned_input: Whether input data is in NED coordinate system
        """
        self.first_message = True
        self.use_monotonic = use_monotonic
        self.is_ned_input = is_ned_input

        # 1. Initialize ROS publisher: fixed topic /mavros/vision_pose/pose
        self.vision_pose_pub = rospy.Publisher(
            "/mavros/vision_pose/pose",  # External pose topic for PX4
            PoseStamped, 
            queue_size=10
        )

        # Print initialization info
        print(f"=== Mavros Vision Pose Publisher Initialized ===")
        print(f"eCAL subscription topic: {ecal_topic}")
        print(f"ROS publishing topic: /mavros/vision_pose/pose")
        print(f"Input coordinate system (NED): {self.is_ned_input}")
        print(f"Use monotonic timestamp: {self.use_monotonic}")

        # 2. Initialize eCAL subscriber and bind callback
        self.ecal_sub = ByteSubscriber(ecal_topic)
        self.ecal_sub.set_callback(self.ecal_msg_callback)

    def ned_to_enu(self, x_ned, y_ned, z_ned, q_ned_w, q_ned_x, q_ned_y, q_ned_z):
        """Convert NED coordinate system to ENU (required by PX4).
        
        Position: x_ENU = y_NED, y_ENU = x_NED, z_ENU = -z_NED
        Orientation: Rotation matrix transformation from NED to ENU
        
        Args:
            x_ned, y_ned, z_ned: Position in NED
            q_ned_w, q_ned_x, q_ned_y, q_ned_z: Quaternion in NED
            
        Returns:
            Tuple of (x_enu, y_enu, z_enu, q_enu_w, q_enu_x, q_enu_y, q_enu_z)
        """
        # Position conversion
        x_enu = y_ned
        y_enu = x_ned
        z_enu = -z_ned

        # Orientation conversion: NED quaternion → ENU quaternion
        # Rotation matrix: R_enu_ned = [[0,1,0],[1,0,0],[0,0,-1]]
        R_ned_to_enu = np.array([[0, 1, 0],
                                 [1, 0, 0],
                                 [0, 0, -1]])
        # Convert NED quaternion to rotation matrix
        R_ned = tf.transformations.quaternion_matrix([q_ned_x, q_ned_y, q_ned_z, q_ned_w])[:3, :3]
        # Apply ENU transformation
        R_enu = R_ned @ R_ned_to_enu.T
        # Convert back to quaternion (x,y,z,w order)
        q_enu = tf.transformations.quaternion_from_matrix(
            np.vstack((np.hstack((R_enu, np.zeros((3,1)))), [0,0,0,1]))
        )
        q_enu_w, q_enu_x, q_enu_y, q_enu_z = q_enu[3], q_enu[0], q_enu[1], q_enu[2]

        return x_enu, y_enu, z_enu, q_enu_w, q_enu_x, q_enu_y, q_enu_z
    
    def ned_to_enu_1(self, x_ned, y_ned, z_ned, q_ned_w, q_ned_x, q_ned_y, q_ned_z):
        """Alternative NED to ENU conversion with corrected rotation logic.
        
        Position: x_ENU = y_NED, y_ENU = x_NED, z_ENU = -z_NED
        Orientation: NED quaternion → ENU quaternion with corrected rotation
        
        Args:
            x_ned, y_ned, z_ned: Position in NED
            q_ned_w, q_ned_x, q_ned_y, q_ned_z: Quaternion in NED
            
        Returns:
            Tuple of (x_enu, y_enu, z_enu, q_enu_w, q_enu_x, q_enu_y, q_enu_z)
        """
        # 1. Position conversion (logic is correct, keep it)
        x_enu = y_ned
        y_enu = x_ned
        z_enu = -z_ned

        # 2. Orientation conversion: corrected rotation logic
        # Step 1: NED orientation → body orientation (rotate 180° around X axis)
        # Quaternion for 180° rotation around X axis (x,y,z,w): (1, 0, 0, 0)
        q_rot_x180 = np.array([1.0, 0.0, 0.0, 0.0])  # x,y,z,w
        # Step 2: Rotate -90° around Z axis (clockwise) to align NED and ENU XY axes
        # Quaternion for -90° rotation around Z axis (x,y,z,w): (0, 0, -√2/2, √2/2)
        q_rot_z_neg90 = np.array([0.0, 0.0, -0.70710678, 0.70710678])  # x,y,z,w

        # Convert NED quaternion to (x,y,z,w) format (consistent with tf)
        q_ned = np.array([q_ned_x, q_ned_y, q_ned_z, q_ned_w])

        # Orientation rotation: multiply by X 180° first, then Z -90°
        q_temp = tf.transformations.quaternion_multiply(q_ned, q_rot_x180)
        q_enu = tf.transformations.quaternion_multiply(q_temp, q_rot_z_neg90)

        # Extract ENU quaternion w,x,y,z (tf output is x,y,z,w)
        q_enu_x, q_enu_y, q_enu_z, q_enu_w = q_enu[0], q_enu[1], q_enu[2], q_enu[3]

        return x_enu, y_enu, z_enu, q_enu_w, q_enu_x, q_enu_y, q_enu_z


    def ecal_msg_callback(self, topic_name, msg, time_ecal):
        """eCAL message callback: parse → convert → publish to /mavros/vision_pose/pose."""
        # Parse eCAL Odometry3d message
        with eCALOdometry3d.Odometry3d.from_bytes(msg) as odom_msg:
            # Print metadata on first message
            if self.first_message:
                print(f"eCAL message info:")
                print(f"  Reference frame: {odom_msg.referenceFrame}")
                print(f"  Body frame: {odom_msg.bodyFrame}")
                print(f"  Velocity frame: {odom_msg.velocityFrame}")
                self.first_message = False

            # Print data every 100 frames (for debugging, can be commented out)
            if odom_msg.header.seq % 100 == 0:
                print(f"\nFrame sequence: {odom_msg.header.seq}")
                print(f"Device latency: {odom_msg.header.latencyDevice / 1e6:.2f} ms")
                print(f"NED raw position: ({odom_msg.pose.position.x:.2f}, {odom_msg.pose.position.y:.2f}, {odom_msg.pose.position.z:.2f})")

            # --------------------------
            # 1. Initialize PoseStamped message
            # --------------------------
            pose_stamped = PoseStamped()

            # Timestamp: prefer using eCAL message timestamp (converted to ROS time)
            if self.use_monotonic:
                # If using monotonic time, convert from eCAL stamp (ns)
                pose_stamped.header.stamp = rospy.Time.from_sec(odom_msg.header.stamp / 1e9)
            else:
                # Otherwise use current ROS time (not recommended due to potential latency)
                pose_stamped.header.stamp = rospy.Time.now()

            # Frame ID: fixed to "odom" (PX4 default reference frame, matches MAVROS config)
            pose_stamped.header.frame_id = "odom"

            # --------------------------
            # 2. Data mapping and coordinate system conversion
            # --------------------------
            # Extract NED pose from eCAL
            x_ned = odom_msg.pose.position.x
            y_ned = odom_msg.pose.position.y
            z_ned = odom_msg.pose.position.z
            q_ned_w = odom_msg.pose.orientation.w
            q_ned_x = odom_msg.pose.orientation.x
            q_ned_y = odom_msg.pose.orientation.y
            q_ned_z = odom_msg.pose.orientation.z

            # If input is NED, convert to ENU required by PX4
            if self.is_ned_input:
                x, y, z, q_w, q_x, q_y, q_z = self.ned_to_enu(
                    x_ned, y_ned, z_ned, q_ned_w, q_ned_x, q_ned_y, q_ned_z
                )
            else:
                # If already ENU, use directly
                x, y, z = x_ned, y_ned, z_ned
                q_w, q_x, q_y, q_z = q_ned_w, q_ned_x, q_ned_y, q_ned_z

            # Map to pose message
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = z
            pose_stamped.pose.orientation.w = q_w
            pose_stamped.pose.orientation.x = q_x
            pose_stamped.pose.orientation.y = q_y
            pose_stamped.pose.orientation.z = q_z

            # --------------------------
            # 3. Publish message to /mavros/vision_pose/pose
            # --------------------------
            self.vision_pose_pub.publish(pose_stamped)


def main():
    """Main entry point for eCAL to MAVROS vision pose bridge."""
    # Print eCAL version information
    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))

    # --------------------------
    # Parse command line arguments
    # --------------------------
    parser = argparse.ArgumentParser(
        description="Subscribe to eCAL pose and publish to /mavros/vision_pose/pose"
    )
    parser.add_argument(
        '--ecal_topic', 
        type=str, 
        default="S1/vio_odom", 
        help="eCAL subscription topic (default: S1/vio_odom)"
    )
    parser.add_argument(
        '--is_ned_input', 
        action="store_true", 
        help="Whether input data is in NED coordinate system (default: False, i.e., ENU)"
    )
    parser.add_argument(
        '--monotonic_time', 
        action="store_true", 
        help="Whether to use eCAL message's monotonic timestamp (recommended: True)"
    )
    args = parser.parse_args()

    # --------------------------
    # Initialize eCAL and ROS
    # --------------------------
    # Initialize eCAL
    ecal_core.initialize(sys.argv, "mavros_vision_pose_bridge")
    ecal_core.set_process_state(1, 1, "Running")

    # Initialize ROS node (unique name to avoid conflicts)
    rospy.init_node("ecal_to_mavros_vision_pose", anonymous=True)

    # --------------------------
    # Start publisher and block
    # --------------------------
    # Create publisher instance
    publisher = MavrosVisionPosePublisher(
        ecal_topic=args.ecal_topic,
        use_monotonic=args.monotonic_time,
        is_ned_input=args.is_ned_input
    )

    # ROS blocking (keep node running)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("\nUser terminated program")
    finally:
        # Release eCAL resources
        ecal_core.finalize()
        print("eCAL resources released")


if __name__ == "__main__":
    main()
