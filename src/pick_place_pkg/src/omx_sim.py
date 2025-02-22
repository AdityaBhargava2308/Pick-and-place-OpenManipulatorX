#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger
import pybullet as p
import pybullet_data
import time
from os.path import join
from ament_index_python.packages import get_package_share_directory
import random

class PyBulletRobotNode(Node):
    def __init__(self):
        super().__init__('pybullet_robot_node')

        # Constants
        self.END_EFFECTOR_INDEX = 7
        self.RESET_JOINT_INDICES = [1, 2, 3, 4, 5, 6]
        self.RESET_JOINT_VALUES = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.SIMULATION_TIME_STEP = 1.0/240


        # PyBullet setup
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)
        p.setPhysicsEngineParameter(numSolverIterations=150)
        p.setRealTimeSimulation(False)
        self.planeID = p.loadURDF("plane.urdf")
        p.setTimeStep(self.SIMULATION_TIME_STEP)

        robotUrdfPath = join(get_package_share_directory('pick_place_pkg'), 'urdf', 'omx.urdf')
        self.robotID = p.loadURDF(robotUrdfPath)

        self.omx_joints = [1, 2, 3, 4]
        self.gripper_joints = [5, 6]
        self.movable_joints = self.omx_joints + self.gripper_joints

        # Reset robot
        for i, value in zip(self.RESET_JOINT_INDICES, self.RESET_JOINT_VALUES):
            p.resetJointState(self.robotID, i, value)

        # ROS2 setup
        self.target_joint_positions = [0.0] * 6  # 4 joints + 2 gripper joints
        self.joint_trajectory_sub = self.create_subscription(JointTrajectory,'/command_joint_trajectory',self.joint_trajectory_callback,10)
        self.ee_pose_pub = self.create_publisher(Pose, '/ee_pose', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Object spawning service
        self.spawn_service = self.create_service(Trigger, '/spawn_object', self.spawn_object_callback)
        self.objUrdfPath = join(get_package_share_directory('pick_place_pkg'), 'urdf', 'cylinder.urdf')

    def joint_trajectory_callback(self, msg):
        positions = msg.points[0].positions.tolist()
        self.target_joint_positions = positions[:4] + [positions[4], positions[4]]

    def spawn_object_callback(self, request, response):
        try:
            x=0.2
            y=0.2
            z = 0.01

            objectID = p.loadURDF(self.objUrdfPath, [x, y, z])
            if objectID != -1:
                self.get_logger().info(f"Object spawned at position: [{x}, {y}, {z}]")
                response.success = True
                response.message = "Object spawned successfully"
            else:
                response.success = False
                response.message = "Failed to spawn object"
        except Exception as e:
            self.get_logger().error(f"Error spawning object: {str(e)}")
            response.success = False
            response.message = f"Error: {str(e)}"

        return response

    def run(self):
        while rclpy.ok():
            start_time = time.time()

            # Process callbacks
            rclpy.spin_once(self, timeout_sec=0)

            # Update simulation
            forces = [5] * len(self.omx_joints) + [1000]*len(self.gripper_joints)
            positionGains = [0.04] * len(self.movable_joints)
            p.setJointMotorControlArray(
                bodyUniqueId=self.robotID,
                jointIndices=self.movable_joints,
                controlMode=p.POSITION_CONTROL,
                targetPositions=self.target_joint_positions,
                forces=forces,
                positionGains=positionGains)
            
            p.stepSimulation()

            # Get and publish end-effector pose
            ee_pos, ee_rot, _, _, _, _ = p.getLinkState(self.robotID, self.END_EFFECTOR_INDEX)
            ee_pose = Pose()
            ee_pose.position.x, ee_pose.position.y, ee_pose.position.z = ee_pos
            ee_pose.orientation.x, ee_pose.orientation.y, ee_pose.orientation.z, ee_pose.orientation.w = ee_rot
            self.ee_pose_pub.publish(ee_pose)

            # Get and publish joint states
            joint_states = p.getJointStates(bodyUniqueId=self.robotID, jointIndices=self.movable_joints)[:5]
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
            joint_state_msg.position = [state[0] for state in joint_states]
            joint_state_msg.velocity = [state[1] for state in joint_states]
            joint_state_msg.effort = [state[3] for state in joint_states]
            self.joint_state_pub.publish(joint_state_msg)

            # Calculate sleep time to maintain desired simulation rate
            elapsed_time = time.time() - start_time
            sleep_time = max(0, self.SIMULATION_TIME_STEP - elapsed_time)
            time.sleep(sleep_time)

        # Cleanup
        p.disconnect()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PyBulletRobotNode()
    node.run()

if __name__ == '__main__':
    main()