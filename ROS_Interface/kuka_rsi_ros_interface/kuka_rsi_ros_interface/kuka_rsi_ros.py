#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor # Essential for background thread + callbacks
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import math
# ROS 2 Messages
from std_msgs.msg import Int32, Bool, Float64MultiArray
from geometry_msgs.msg import PoseStamped, Pose, Quaternion # Assuming Pose for command input
from test_package_interfaces.msg import Correction
from test_package_interfaces.srv import CorrSend, RoutineSend, StopSend# Assuming this is the service for sending corrections
# Math/TF - Using scipy as an example for transformations
from math import radians, degrees
from scipy.spatial.transform import Rotation as R # Requires scipy installed (pip install scipy)

# Python Standard Libraries
import socket
import xml.etree.ElementTree as ET
import threading
import time
import numpy as np
import datetime
# import logging # Can primarily use ROS 2 logging

class KukaRsiNode(Node):
    def __init__(self):
        super().__init__('kuka_rsi_node')

        # --- Parameters ---
        self.host = '10.10.10.20'
        self.port = 59152
        self.robot_frame_id = 'base_link'
        self.position_gain = 0.005
        self.rotation_gain = 0.005
        self.max_correction_position = 0.05
        self.max_correction_angle = 0.025
        
        
        # Waypoint corrections: Loading dicts directly isn't standard.
        # Best practice: Load from a YAML file specified by a parameter path.
        # Simple approach for now: hardcode default, but mention parameterization possibility.
        # self.declare_parameter('waypoints_yaml_path', '/path/to/default.yaml')

        
        # TODO: Implement loading waypoint_corrections from YAML parameter
        self.waypoint_corrections = {
            1: {"X": 0.0, "Y": 0.0, "Z": 0.0, "A": 0.0, "B": 0.0, "C": 0.0},
            2: {"X": 0.0, "Y": 0.0, "Z": 0.0, "A": 0.0, "B": 0.0, "C": 0.0},
            3: {"X": 0.0, "Y": 0.0, "Z": 0.0, "A": 0.0, "B": 0.0, "C": 0.0},
            4: {"X": 0.0, "Y": 0.0, "Z": 0.0, "A": 0.0, "B": 0.0, "C": 0.0},
            5: {"X": 0.0, "Y": 0.0, "Z": 0.0, "A": 0.0, "B": 0.0, "C": 0.0},
            6: {"X": 0.0, "Y": 0.0, "Z": 0.0, "A": 0.0, "B": 0.0, "C": 0.0},
            7: {"X": 0.0, "Y": 0.0, "Z": 0.0, "A": 0.0, "B": 0.0, "C": 0.0},
            8: {"X": 0.0, "Y": 0.0, "Z": 0.0, "A": 0.0, "B": 0.0, "C": 0.0},
            9: {"X": 0.0, "Y": 0.0, "Z": 0.0, "A": 0.0, "B": 0.0, "C": 0.0}
        }


        # --- State Variables ---
        self.target_positions = {"X": 0.0, "Y": 0.0, "Z": 0.0, "A": 0.0, "B": 0.0, "C": 0.0}
        self.current_positions = {"X": 0.0, "Y": 0.0, "Z": 0.0, "A": 0.0, "B": 0.0, "C": 0.0}
        self.current_joint_val = {"A1": 0.0, "A2": 0.0, "A3": 0.0, "A4": 0.0, "A5": 0.0, "A6": 0.0} 
        self.correction_val = {"X": 0.0, "Y": 0.0, "Z": 0.0, "A": 0.0, "B": 0.0, "C": 0.0}
        self.initial_corr_pos ={"X": 0.0, "Y": 0.0, "Z": 0.0, "A": 0.0, "B": 0.0, "C": 0.0}
        self.target_mode = True
        
        
        self.connected = False
        self.last_packet_time = 0.0
        self.packet_counter = 0
        self.current_step = 0
        self.last_movecorr_flag = '0'
        self.stop_flag = 0 # Sent to robot
        self.routine_num = 99 # Sent to robot
        self.routine_user = list() # Queue from ROS commands
        self.routine_user_flag = 0 # Internal state machine flag

        self.running = True # Control flag for comm thread
        self.comm_lock = threading.Lock() # Protect shared state variables

        # --- QoS Profiles ---
        qos_profile_status = QoSProfile(
             reliability=QoSReliabilityPolicy.RELIABLE,
             history=QoSHistoryPolicy.KEEP_LAST, depth=1,
             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL) # Latching for status
        qos_profile_reliable = QoSProfile(
             reliability=QoSReliabilityPolicy.RELIABLE,
             history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        qos_profile_best_effort = QoSProfile( # Maybe for high-rate pose if needed
             reliability=QoSReliabilityPolicy.BEST_EFFORT,
             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        

        # --- ROS 2 Publishers ---
        self.pub_current_pose = self.create_publisher(PoseStamped, '~/current_pose',qos_profile_reliable)
        self.pub_current_step = self.create_publisher(Int32, '~/current_step',qos_profile_status)
        self.pub_connection_status = self.create_publisher(Bool, '~/connection_status',qos_profile_status)
        self.pub_joint_val = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', qos_profile_reliable)
        

        # --- ROS 2 Subscribers ---
        self.sub_target_pose = self.create_subscription(Pose, '~/target_pose_cmd', self.target_pose_callback, qos_profile_reliable)
        # self.sub_routine = self.create_subscription(Int32, '~/routine_cmd', self.routine_callback, qos_profile_reliable)
        # self.sub_correction = self.create_subscription(Correction, '~/correction_cmd', self.correction_callback, qos_profile_reliable)
        # self.sub_routine = self.create_subscription(Int32, '~/correction_cmd', self.correction_callback)
        
        # --- ROS 2 Service/Client ---
        self.srv_corr = self.create_service(CorrSend, '~/corr_send', self.correction_callback)
        self.srv_routine = self.create_service(RoutineSend, '~/routine_send', self.routine_callback)
        self.srv_stop = self.create_service(StopSend, '~/stop_send', self.stop_callback)
        
        

        # --- Start Communication Thread ---
        self.get_logger().info("Starting communication thread...")
        self.comm_thread = threading.Thread(target=self.communication_loop, daemon=True)
        self.comm_thread.start()

        self.get_logger().info(f"KUKA RSI node started. Listening for robot on {self.host}:{self.port}")
        self.publish_status() # Publish initial status

    # function to convert the angles from [-180,180] to [0 360]
    def convert_angle_to360(self, angle):
        return angle + 360 if angle < 0 else angle

    # Function to normalize angles to the range [-180, 180]
    def convert_angle_to180(self, angle):
        if angle > 180.0: 
            return angle - 360.0
        elif angle < -180.0: 
            return angle + 360.0 
        else: 
            return angle

    # Function to calculate the shortest angular difference
    def shortest_angle_difference(self, target, current):
        diff = self.convert_angle_to360(target) - self.convert_angle_to360(current)
        # Normalize difference to [-180, 180]
        if diff > 180.0: 
            diff -= 360.0
        elif diff <= -180.0: 
            diff += 360.0 # Use <= to include -180
        return diff

    def calculate_corrections(self):
        """Calculate corrections based on target_positions and waypoint_offsets."""
        corrections = {}
        # Use parameters for gains/limits
        pos_gain = self.position_gain
        rot_gain = self.rotation_gain
        max_pos_corr = self.max_correction_position
        max_rot_corr = self.max_correction_angle

  
        has_manual_target = any(abs(self.target_positions[axis]) != 0.0 for axis in self.target_positions) # Check non-zero target
        
        for axis in ["X", "Y", "Z"]:
            if has_manual_target:
                
                #  If manual target exists, apply it

                combined_target = self.target_positions[axis] 
                diff = combined_target - self.current_positions[axis]
                
            else:
                # If no manual target, just apply waypoint correction directly
                diff = 0

            corrections[axis] = np.clip(pos_gain * diff, -max_pos_corr, max_pos_corr)

        for axis in ["A", "B", "C"]:
            
            if has_manual_target:
                combined_target = self.target_positions[axis]
                # For rotational axes with manual target
                diff_angle = self.shortest_angle_difference(combined_target, self.current_positions[axis])
            else:
                # For rotational axes without manual target, use waypoint correction
                diff_angle = 0
                
            correction_value = np.clip(rot_gain * diff_angle, -max_rot_corr, max_rot_corr)
            
            
            corrections[axis] = correction_value if axis == "A" else -correction_value

        return corrections


    def correction_send(self):
        """Calculate corrections using proportional control for smoother movement."""
        
        corrections = {"X": 0.0, "Y": 0.0, "Z": 0.0, "A": 0.0, "B": 0.0, "C": 0.0}
        pos_gain = self.position_gain 
        rot_gain = self.rotation_gain
        max_pos_corr = self.max_correction_position
        max_rot_corr = self.max_correction_angle
        active_axis = None

        # Find the single active correction axis 
        for axis, val in self.correction_val.items():
             if abs(val) != 0.0:
                 active_axis = axis
                 break

        if active_axis:
            axis = active_axis # Use the found axis
            target_val = 0.0
            if axis in ["X", "Y", "Z"]:
                target_val = self.correction_val[axis] + self.initial_corr_pos[axis]
                diff = target_val - self.current_positions[axis]
                corrections[axis] = np.clip(pos_gain * diff, -max_pos_corr, max_pos_corr)
            elif axis in ["A", "B", "C"]:
                target_val = self.convert_angle_to180(self.correction_val[axis] + self.initial_corr_pos[axis])
                diff_angle = self.shortest_angle_difference(target_val, self.current_positions[axis])
                correction_value = np.clip(rot_gain * diff_angle, -max_rot_corr, max_rot_corr)
                # Apply KUKA B/C sign inversion if needed
                corrections[axis] = correction_value if axis == "A" else -correction_value
            else:
                 pass 

        return corrections

    def create_response(self, ipoc, corrections):
        """Generate XML response string."""
        # Ensure all axes have a value, default to 0.0 if missing

        if self.last_movecorr_flag == '0':

            corr_x = 0.0
            corr_y = 0.0
            corr_z = 0.0
            corr_a = 0.0
            corr_b = 0.0
            corr_c = 0.0
            self.correction_val = {"X": 0.0, "Y": 0.0, "Z": 0.0, "A": 0.0, "B": 0.0, "C": 0.0}
            

        else:

            corr_x = corrections.get("X", 0.0)
            corr_y = corrections.get("Y", 0.0)
            corr_z = corrections.get("Z", 0.0)
            corr_a = corrections.get("A", 0.0)
            corr_b = corrections.get("B", 0.0)
            corr_c = corrections.get("C", 0.0)



        # Format with 4 decimal places
        response = (
            f'<Sen Type="ImFree"><EStr>ROS2 Path Controller</EStr>'
            f'<Tech T21="1.09" T22="2.08" T23="3.07" T24="4.06" T25="5.05" T26="6.04" T27="7.03" T28="8.02" T29="9.01" T210="10.00" />'
            f'<RKorr X="{corr_x:.4f}" Y="{corr_y:.4f}" Z="{corr_z:.4f}" '
            f'A="{corr_a:.4f}" B="{corr_b:.4f}" C="{corr_c:.4f}" />'
            f'<Routine>{self.routine_num}</Routine><DiO>{self.stop_flag}</DiO><IPOC>{ipoc}</IPOC></Sen>'
        )
        return response

    # --- ROS 2 Callback Handlers ---
    def target_pose_callback(self, msg: Pose):
        """Callback for receiving target pose commands via geometry_msgs/Pose."""
        self.get_logger().info(f"Received target pose: P=[{msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f}] Q=[{msg.orientation.x:.3f}, ..., {msg.orientation.w:.3f}]")
        with self.comm_lock:
            self.target_mode = True # Receiving a target implies target mode
            
            self.target_positions["X"] = msg.position.x 
            self.target_positions["Y"] = msg.position.y 
            self.target_positions["Z"] = msg.position.z 

            
            try:
                
                self.target_positions["A"] = msg.orientation.z # Z rotation -> KUKA A?
                self.target_positions["B"] = msg.orientation.y # Y rotation -> KUKA B?
                self.target_positions["C"] = msg.orientation.x # X rotation -> KUKA C?

                self.get_logger().debug(f"Target KUKA ABC: A={self.target_positions['A']:.2f}, B={self.target_positions['B']:.2f}, C={self.target_positions['C']:.2f}")
            except Exception as e:
                self.get_logger().error(f"Angle assignment failed: {e}", exc_info=True)
                # Optionally clear orientation target or ignore command
                self.target_positions["A"] = self.current_positions["A"] # Keep current if error
                self.target_positions["B"] = self.current_positions["B"] # Keep current if error
                self.target_positions["C"] = self.current_positions["C"] # Keep current if error

    def correction_callback(self, request: CorrSend, response: CorrSend):
        """Callback for receiving correction pose commands via test_package_interfaces/CorrSend."""
        self.get_logger().info(f"Received correction value: P=[{request.name}, {request.val:.3f}]")

        with self.comm_lock:
            self.target_mode = False # Receiving a target implies target mode
            
            for axis in ["X", "Y", "Z", "A", "B", "C"]:
                self.correction_val[axis] = 0.0 # Reset all corrections
                self.initial_corr_pos[axis] = self.current_positions[axis] # Store current position as initial correction
            
            axis = request.name
            value = request.val
            response.result = 1 # Default success
            self.correction_val[axis] = value # Set the correction value for the specified axis

            self.get_logger().debug(f"Correction value set: {axis}={self.correction_val[axis]:.2f}")

            return response # Return the response object

    
    def routine_callback(self, request: RoutineSend, response: RoutineSend):
        """Callback for receiving routine commands."""
        self.get_logger().info(f"Received routine command: {request.val}")
        with self.comm_lock:
            if isinstance(request.val, int) and request.val >= 0: # Basic validation
                 self.routine_user.append(request.val)
                 self.get_logger().info(f"Added routine {request.val} to queue (queue size: {len(self.routine_user)}).")
            else:
                 self.get_logger().warn(f"Invalid routine command received: {request.val}")

        response.result = 1 # Default success
        return response # Return the response object
    

    def stop_callback(self, request: StopSend, response: StopSend):
        """Callback for receiving stop commands."""
        self.get_logger().info(f"Received stop command:")
        with self.comm_lock:

            print("stopping movecorr.")
            self.stop_flag = 1
            
            

        response.result = 1 # Default success
        return response # Return the response object


    # --- Status Publishing ---
    def publish_status(self):
        """Publish current robot status derived from state variables."""
        # Use comm_lock to ensure consistent read of state variables
        with self.comm_lock:
            connected_status = self.connected
            current_step_msg = Int32(data=self.current_step)
            
            current_positions_local = self.current_positions.copy() # Copy dict for safety

        # Publish status flags (can be done outside lock)
        self.pub_connection_status.publish(Bool(data=connected_status))
        self.pub_current_step.publish(current_step_msg)
        

        # Publish current pose if connected
        if connected_status:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.robot_frame_id

            
            pose_msg.pose.position.x = current_positions_local["X"] 
            pose_msg.pose.position.y = current_positions_local["Y"] 
            pose_msg.pose.position.z = current_positions_local["Z"] 

            
            try:
                # *** CRITICAL: Match KUKA's Euler convention & Mapping ***
                # Example using intrinsic 'zyx' (check KUKA docs!)
                euler_rad = [
                    radians(current_positions_local["A"]), # Map to Z?
                    radians(current_positions_local["B"]), # Map to Y?
                    radians(current_positions_local["C"])  # Map to X?
                ]
                r = R.from_euler('zyx', euler_rad, degrees=False) # Use same convention as in callback
                quat = r.as_quat()
                pose_msg.pose.orientation.x = current_positions_local["C"]
                pose_msg.pose.orientation.y = current_positions_local["B"]
                pose_msg.pose.orientation.z = current_positions_local["A"]
                pose_msg.pose.orientation.w = 1.0
            except Exception as e:
                self.get_logger().warn(f"Angle presentation failed: {e}", throttle_duration_sec=10.0)
                # Publish identity quaternion if conversion fails
                pose_msg.pose.orientation.x = 0.0
                pose_msg.pose.orientation.y = 0.0
                pose_msg.pose.orientation.z = 0.0
                pose_msg.pose.orientation.w = 1.0

            self.pub_current_pose.publish(pose_msg)


    # --- Communication Loop (Runs in a separate thread) ---
    def communication_loop(self):
        """Handles UDP communication with the KUKA RSI."""
        sock = None # Initialize sock to None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind((self.host, self.port))
            sock.settimeout(1.0) # Socket timeout
            self.get_logger().info(f"Communication socket bound to {self.host}:{self.port}")
        except socket.error as e:
            self.get_logger().fatal(f"Failed to bind socket: {e}. Shutting down node.")
            # Signal main thread to shut down gracefully if possible
            if rclpy.ok():
                 rclpy.try_shutdown() # Request shutdown
            self.running = False # Stop trying to run loop
            return # Exit thread

        while self.running and rclpy.ok():
            connected_before = self.connected
            new_step_detected = False
            try:
                # Receive data
                data, addr = sock.recvfrom(1024)
                current_time = self.get_clock().now().nanoseconds / 1e9 # Use ROS time

                # --- Process Received Data (Protected by Lock) ---
                with self.comm_lock:
                    self.last_packet_time = current_time
                    self.packet_counter += 1
                    if not self.connected:
                        self.connected = True
                        self.get_logger().info(f"Connection established with robot: {addr}")

                    received_xml = data.decode('utf-8')
                    try:
                        root = ET.fromstring(received_xml)
                        ipoc_elem = root.find(".//IPOC")

                        if ipoc_elem is not None:
                            ipoc = ipoc_elem.text
                        else:
                            ipoc = "0000000"
                            print("Warning: No IPOC found in received XML")
                        

                        # --- Extract Data from XML ---
                        # Current Position (RIst)
                        rist_elem = root.find(".//RIst")
                        if rist_elem is not None:
                            for axis in self.current_positions.keys():
                                try:
                                    self.current_positions[axis] = float(rist_elem.get(axis, "0.0"))
                                except (ValueError, TypeError):
                                    self.get_logger().warn(f"Failed to parse RIst value for axis {axis}", throttle_duration_sec=10.0)

                        # Current Step
                        step_elem = root.find(".//Step")
                        if step_elem is not None and step_elem.text:
                            try:
                                new_step = int(step_elem.text)
                                if new_step != self.current_step:
                                    self.get_logger().info(f"Waypoint changed: {self.current_step} -> {new_step}")
                                    self.current_step = new_step
                                    
                                    new_step_detected = True # Flag to publish status update
                            except (ValueError, TypeError):
                                self.get_logger().warn(f"Failed to parse Step value: {step_elem.text}", throttle_duration_sec=10.0)

                        # Movecorr Flag (Optional, if used by KRL)
                        movecorr_flag_elem = root.find(".//Movecorr_flag")
                        if movecorr_flag_elem is not None:
                            current_movecorr_flag = movecorr_flag_elem.text

                            if current_movecorr_flag != self.last_movecorr_flag:
                                self.get_logger().info(f"Movecorr_flag changed to: {current_movecorr_flag}")
                                self.last_movecorr_flag = current_movecorr_flag
                                # Reset internal stop flag if KRL stops MOVECORR
                                if self.last_movecorr_flag == "0" and self.stop_flag == 1:
                                    self.get_logger().info("Resetting internal stop_flag as external MOVECORR flag is 0.")
                                    self.stop_flag = 0

                                

                        # Routine Number Logic (AnOut_1)
                        an_out_elem = root.find(".//AnOut_1")
                        if an_out_elem is not None:
                            try:
                                an_out_val = int(an_out_elem.text)
                                # Logic to handle routine state machine
                                if an_out_val == 99 and self.routine_num != 99 and self.routine_user_flag == 0:
                                    self.routine_num = 99
                                    self.get_logger().info("Routine number is set to 99")
                                elif len(self.routine_user) > 0 and self.routine_num == 99:
                                    self.routine_num = self.routine_user.pop(0)
                                    self.routine_user_flag = 1 # Mark as user-initiated
                                    self.get_logger().info(f"Routine number is set to  {self.routine_num}")
                                elif self.routine_user_flag == 1 and self.routine_num == an_out_val:
                                    # Robot acknowledged command, clear flag (or wait for 99?)
                                    self.routine_user_flag = 0
                                    self.get_logger().info(f"Routine flag is off")
                            except (ValueError, TypeError):
                                self.get_logger().warn(f"Failed to parse AnOut_1 value: {an_out_elem.text}", throttle_duration_sec=10.0)

                        joint_elem = root.find(".//AIPos")  
                        if joint_elem is not None:
                            for joint in ["A1", "A2", "A3", "A4", "A5", "A6"]:
                                try:
                                    self.current_joint_val[joint] = float(joint_elem.get(joint, "0.0"))
                                    joint_values_rad = [math.pi / 180 * val for val in self.current_joint_val.values()]
                                except (ValueError, TypeError):
                                    self.get_logger().warn(f"Failed to parse AIPos value for axis {joint}", throttle_duration_sec=10.0)
                            self.pub_joint_val.publish(Float64MultiArray(data=joint_values_rad))
                        # --- Calculate Corrections (inside lock to read consistent state)---
                        if self.target_mode:
                            corrections = self.calculate_corrections()
                        else:
                            corrections = self.correction_send()
                            # Consider resetting correction_val and target_mode here?

                        # --- Create and Send Response ---
                        response_xml = self.create_response(ipoc, corrections)
                        sock.sendto(response_xml.encode('utf-8'), addr)

                    except ET.ParseError as e:
                        self.get_logger().error(f"XML Parse Error: {e}. Received: {received_xml[:200]}...")
                        # Don't send response for bad XML
                    except Exception as e_inner:
                         # Catch other errors during processing within the lock
                         self.get_logger().error(f"Error processing received data: {e_inner}", exc_info=True)

                # --- End Lock ---

            except socket.timeout:
                # Handle socket timeout - check if connection truly lost
                time_now = self.get_clock().now().nanoseconds / 1e9
                if self.connected and (time_now - self.last_packet_time > 5.0): # 5 second timeout threshold
                    self.get_logger().warn("Connection to robot timed out.")
                    with self.comm_lock: # Need lock to change connected status
                        self.connected = False
                    # Publish status outside lock
                # Continue loop after timeout
                continue

            except socket.error as e:
                # Handle other socket errors (e.g., connection refused if robot resets)
                self.get_logger().error(f"Socket Error: {e}. Assuming disconnected.")
                with self.comm_lock:
                    self.connected = False
                # Potentially try to re-bind or wait? For now, just log and assume disconnected.
                time.sleep(1.0) # Prevent spamming logs on persistent socket errors

            except Exception as e_outer:
                # Catch unexpected errors in the loop
                 if self.running and rclpy.ok(): # Avoid logging during shutdown
                     self.get_logger().error(f"Unhandled error in communication loop: {e_outer}", exc_info=True)
                     time.sleep(0.5) # Small delay before next iteration

            # --- Publish Status (Outside lock, after processing/reply) ---
            try:
                 self.publish_status()
            except Exception as pub_e:
                 self.get_logger().error(f"Error during status publishing: {pub_e}", exc_info=True)


        # --- Loop Exit Cleanup ---
        if sock:
            sock.close()
        self.running = False # Ensure flag is false
        self.get_logger().info("Communication loop stopped.")
        with self.comm_lock: # Ensure final disconnected status is set
            self.connected = False
        self.publish_status() # Publish final disconnected status


    def destroy_node(self):
        """Custom cleanup on node destruction."""
        self.get_logger().info("Shutting down KUKA RSI Node...")
        self.running = False # Signal communication thread to stop
        if hasattr(self, 'comm_thread') and self.comm_thread.is_alive():
            self.get_logger().info("Waiting for communication thread to finish...")
            self.comm_thread.join(timeout=2.0) # Wait for thread
            if self.comm_thread.is_alive():
                self.get_logger().warn("Communication thread did not exit cleanly.")
        super().destroy_node() # Call parent cleanup
        self.get_logger().info("Node shutdown complete.")

def main(args=None):
    rclpy.init(args=args)
    node = None # Initialize to prevent reference error in finally
    executor = None
    try:
        node = KukaRsiNode()
        # Use MultiThreadedExecutor to handle callbacks and allow background thread
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        node.get_logger().info("Starting executor spin...")
        executor.spin() # Blocks until shutdown
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt, shutting down.')
    except Exception as e:
        if node:
             node.get_logger().fatal(f'Unhandled exception: {e}', exc_info=True)
        else:
             print(f'Unhandled exception before node init: {e}') # Basic print if logger not available
    finally:
        # Cleanup
        if executor:
            executor.shutdown()
        if node:
            node.destroy_node() # Ensure cleanup method is called
        if rclpy.ok():
            rclpy.shutdown()
        print("ROS 2 Shutdown complete.")


if __name__ == '__main__':
    main()
