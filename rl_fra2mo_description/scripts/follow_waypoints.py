#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from math import cos
from math import sin
from math import atan2
from math import asin
import math
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def pose_callback(msg, tf_buffer, node):
    try:
        # Recupera la trasformazione da 'map' a 'camera_link_optical'
        transform = tf_buffer.lookup_transform(
            'map',              # Frame di destinazione
            msg.header.frame_id,  # Frame di origine (camera_link_optical)
            rclpy.time.Time(),    # Tempo corrente
            timeout=Duration(seconds=1.0)  # Timeout
        )

        # Trasforma la pose
        transformed_pose = tf_buffer.transform(msg, 'map')

        x = transformed_pose.pose.position.x
        y = transformed_pose.pose.position.y
        z = transformed_pose.pose.position.z

        qx = transformed_pose.pose.orientation.x
        qy = transformed_pose.pose.orientation.y
        qz = transformed_pose.pose.orientation.z
        qw = transformed_pose.pose.orientation.w

        R= atan2(2*(qw*qx+qy*qz), 1-2*(qx**2 + qy**2))
        P= asin(2*(qw*qy-qz*qx))
        Y= atan2(2*(qw*qz+qx*qy), 1-2*(qy**2 + qz**2))

        # Log delle componenti estratte
        #node.get_logger().info(f"Position: x={x}, y={y}, z={z}")
        #node.get_logger().info(f"Orientation: R={R}, P={P}, Y={Y}")

        # Qui puoi modificare la pose come necessario
        # Esempio: aggiungere un offset alla posizione
        compensation={"x": -3 , "y": 3.5, "z":0.1, "Y": -1.570796}
        x_modified = y + compensation.get("x")   # Offset di esempio
        y_modified = compensation.get("y") - x  # Offset di esempio
        z_modified = z + compensation.get("z")
        R_modified = R
        P_modified = P 
        Y_modified = Y + compensation.get("Y")

        node.get_logger().info(f"Modified Position: x={x_modified}, y={y_modified}, z={z_modified}")
        node.get_logger().info(f"Modified orientation: R={R_modified}, P={P_modified}, Y={Y_modified}")

        
        t = TransformStamped()

        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id = 'aruco_marker_frame'
        t.child_frame_id = 'map_gazebo_frame'

        t.transform.translation.x = x_modified
        t.transform.translation.y = y_modified
        t.transform.translation.z = z_modified
        quat = quaternion_from_rpy(R_modified, P_modified, Y_modified)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        node.tf_static_broadcaster.sendTransform(t)

    except Exception as e:
        node.get_logger().error(f"Errore nella trasformazione: {e}")


def quaternion_from_rpy(roll, pitch, yaw):
   
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return x, y, z, w


def main():
    rclpy.init()
    navigator = BasicNavigator()

    choice = 0
    while choice not in [2, 3, 4]:
        choice = int(input("Insert your choise (2/3/4): "))
   

    init_pose_rob={"x": -3 , "y": 3.5, "Y": -1.570796}
    fra2mo_description_path = os.path.join(get_package_share_directory('rl_fra2mo_description'))
    if choice == 2:
        waypoints_path = os.path.join(fra2mo_description_path,"config","waypoints2.yaml")
    elif choice == 3:
        waypoints_path = os.path.join(fra2mo_description_path,"config","waypoints3.yaml")
    elif choice == 4:
        waypoints_path = os.path.join(fra2mo_description_path,"config","waypoints4.yaml")
    

    try:
    # Leggi il file YAML
    	with open(waypoints_path, 'r') as yaml_file:
        	goals_data = yaml.safe_load(yaml_file)
    except FileNotFoundError:
        print(f"File non trovato: {scripts_path}")
    except yaml.YAMLError as e:
    	print(f"Errore durante il parsing del file YAML: {e}")
    
    
    def create_pose(transform):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = init_pose_rob.get("y") -transform["position"]["y"]
        pose.pose.position.y = transform["position"]["x"] - init_pose_rob.get("x")
        pose.pose.position.z = transform["position"]["z"]
        Y_tot = transform["orientation"]["Y"] - init_pose_rob.get("Y")
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = sin(Y_tot/2)
        pose.pose.orientation.w = cos(Y_tot/2)
        return pose

    if choice == 2:
        goals_order = [2,3,1,0]
    elif choice == 3:
        goals_order = [0,1,2,3,4,5,6,7]
    elif choice == 4:
        goals_order = [0,0,1]

    goal_poses = [create_pose(goals_data["waypoints"][i]) for i in goals_order]


    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer="smoother_server")

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################
        
        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()           
            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600):
                navigator.cancelTask()

            if choice == 4 and (feedback.current_waypoint+1) == 2:
                node = Node('aruco_to_map_transformer')
                tf_buffer = Buffer()
                tf_listener = TransformListener(tf_buffer, node)
                node.get_logger().info("In attesa di trasformazioni e pose ArUco...")

                start_time = node.get_clock().now()
                duration_limit = Duration(seconds=10)

                node.tf_static_broadcaster = StaticTransformBroadcaster(node)
                #Sottoscrizione al topic 'aruco_single/pose'
                node.create_subscription(
                    PoseStamped,
                    '/aruco_single/pose',
                    lambda msg: pose_callback(msg, tf_buffer, node),  # Passa TF buffer e nodo al callback
                    10  # Dimensione della coda
                )
                try:
                    while (node.get_clock().now() - start_time) < duration_limit:
                        rclpy.spin_once(node, timeout_sec=0.1)  
                except KeyboardInterrupt:
                    node.get_logger().info("Interruzione manuale con Ctrl+C.")
                finally:
                    #node.destroy_node()
                    print("Nodo terminato correttamente dopo il timeout.")

               
    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeded!')
        current_time = navigator.get_clock().now()
        current_time_reale = current_time-nav_start
        print(current_time_reale)               
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()








    

