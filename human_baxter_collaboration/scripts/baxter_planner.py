#!/usr/bin/env python

import sys
import rospy
import tf2_ros
import geometry_msgs
import moveit_commander

from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from human_baxter_collaboration.msg import BaxterTrajectory
from human_baxter_collaboration.srv import Plan, PlanResponse

## left and right joint name
left_joint_names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
right_joint_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']

## left and right position used for the start state
left_position = None
right_position = None

## left and right baxter group
left_group = None
right_group = None

def jointStateCB(joint):
    """
        Callback function used to read current right and left joint states.
        The values read from unity are stored into two global variables.

        @param:
            - joint: JointState of right and left baxter arm
    """
    global right_position, left_position

    # store joint positions
    right_position = [joint.position[1],joint.position[2],joint.position[3],joint.position[4],joint.position[5],joint.position[6],joint.position[7]]
    left_position = [joint.position[8],joint.position[9],joint.position[10],joint.position[11],joint.position[12],joint.position[13],joint.position[14]]

def setStartJointPosition(arm):
    """
        Function used to set start joint position of the arm passed as parameter.
        Set the current state using the joint position read from the callback.

        @param:
            - arm: arm that has to be set, it could be "right" or "left"
    """

    global left_joint_names, right_joint_names, left_position, right_position, left_group, right_group

    # set the current joint state to the robot
    current_joint_state = JointState()
    if(arm == "left"):
        current_joint_state.name = left_joint_names
        current_joint_state.position = left_position
    elif(arm == "right"):
        current_joint_state.name = right_joint_names
        current_joint_state.position = right_position

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state

    # set the start state to the correct arm
    if(arm == "left"):
        left_group.clear_pose_targets()
        left_group.set_start_state(moveit_robot_state)
    elif(arm == "right"):
        right_group.clear_pose_targets()
        right_group.set_start_state(moveit_robot_state)

def planning(request):
    """
        Service function used to plan a trajectory receiving as request
        the arm and the final position.
        if a plan is found, the service return the BaxterTrajectory obtained

        @param:
            - request: contains the arm that should be used to reach
                       the goal position and the target position.

        @return:
            - baxter_trajectory:
                       contains the arm and the RobotTrajectory used to
                       reach the goal position
    """
    global left_group, right_group
    
    # set start joint position to the arm used
    arm = request.arm.data
    setStartJointPosition(arm)

    # store the goal position into a Pose() variable
    target = geometry_msgs.msg.Pose()
    target.position.x = request.transform.transform.translation.x
    target.position.y = request.transform.transform.translation.y
    target.position.z = request.transform.transform.translation.z
    target.orientation.x = request.transform.transform.rotation.x
    target.orientation.y = request.transform.transform.rotation.y
    target.orientation.z = request.transform.transform.rotation.z
    target.orientation.w = request.transform.transform.rotation.w

    # plan with the interested arm
    if(arm == "left"):
        left_group.set_pose_target(target)
        success, trajectory, val, error_msg = left_group.plan()

    elif(arm == "right"):
        right_group.set_pose_target(target)
        success, trajectory, val, error_msg = right_group.plan()

    # if a plan is found, the service return the BaxterTrajectory obtained 
    if(success):
        baxter_trajectory = BaxterTrajectory(arm,[trajectory])
        return PlanResponse(baxter_trajectory)
    else:
        rospy.loginfo("Plan not found!")

def main():
    """
        Main function that sets the subscriber to the joint state, create
        the service that manages the request for planning, initializes
        the two moveit group, one for each arm and sets the table as
        constraint of the scene.
    """

    global left_group, right_group

    rospy.init_node('baxter_planner')
    moveit_commander.roscpp_initialize(sys.argv)

    # subscriber to Baxter joint states
    rospy.Subscriber('baxter_joint_states', JointState, jointStateCB)
    # create the service planner
    rospy.Service('baxter_planner_service', Plan, planning)

    left_group = moveit_commander.MoveGroupCommander("left_arm")
    right_group = moveit_commander.MoveGroupCommander("right_arm")

    rospy.sleep(2)

    # adding the table as constraint
    scene_inter = moveit_commander.PlanningSceneInterface()
    table = geometry_msgs.msg.PoseStamped()
    table.header.frame_id = "world"
    table.pose.position.x = 0.76
    table.pose.position.y = 0.0
    table.pose.position.z = 0.4
    table.pose.orientation.w = 1.0
    table_name = "table"
    scene_inter.add_box(table_name, table, size=(0.6, 2.0, 0.7))

    rospy.spin()

if __name__ == "__main__":
    main()
