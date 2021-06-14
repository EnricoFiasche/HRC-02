#!/usr/bin/env python

import sys
import rospy
import tf2_ros
import moveit_commander

from std_msgs.msg import Bool, String
from human_baxter_collaboration.srv import Plan, CollisionAvoidance, CollisionAvoidanceResponse 
from human_baxter_collaboration.msg import BaxterTrajectory

## variable used to know when the goal is reached
canGo = None 

## variable used to know when a collision is detected
collisionDetected = None

## BaxterTrajectory publisher
trajPub = None

## publisher used to close and open the gripper
gripperPub = None

## publisher used to put baxter in the rest position
restPub = None

## publisher used to stop a BaxterTrajectory
stopPub = None

## client used to plan a trajectory
planner_client = None

## transform buffer used to read the position sent by the tf adapter 
tfBuffer = None

def collisionService(request):
    """
        Service function called when a collision is detected.

        @param:
            - request: true value, which indicates a possible collision

        @return:
            - CollisionAvoidanceResponse(response):
                       response of the collision avoidance
    """
    global collisionDetected, stopPub

    response = Bool()
    response.data = True

    # if a possible collision is detected
    if(request.request.data):
        # stop the trajectory
        rospy.loginfo("Collision detected!")
        collisionDetected = True
        stopPub.publish(response)

    return CollisionAvoidanceResponse(response)


def targetReachedCB(reached):
    """
        Callback function which is called when baxter reaches a
        target position

        @param:
            - reached: Baxter reached the target position
    """
    global canGo

    # Baxter now can go to the next position
    canGo = True

def startSimulation():
    """
        Function used to read and publish the next goal position, close and open the gripper
        checking if a collision is detected.
        This function publishes four different trajectory:
        - pre-grasping trajectory (15 cm above the block) with gripper open
        - grasp trajectory with gripper closed
        - lift-up trajectory (15 cm above the block)
        - final position (bluebox or middlePlacement) with gripper open

        If a collision is detected, it will wait 2 seconds, then try again to reach the
        final position.
    """
    global canGo, trajPub, gripperPub, restPub, planner_client, tfBuffer, collisionDetected

    isClosed = Bool()
    arm = String()

    # blueblox position
    bluebox = tfBuffer.lookup_transform('world','Bluebox',rospy.Time())
    bluebox.transform.translation.z += 0.10

    # middle position
    middlePos = tfBuffer.lookup_transform('world','MiddlePlacementN',rospy.Time())
    middlePos.transform.translation.z += 0.05

    # a priori known list of object that should be picked and the final position where should be placed
    order = [
             ['C',bluebox],
             ['E',middlePos],
             ['MiddlePlacementN',bluebox],
             ['G',bluebox],
             ['I',bluebox],
             ['M',middlePos],
             ['MiddlePlacementN',bluebox],
            ]

    # for each object in order
    for object in order:
        rospy.loginfo("Going to grasp block "+object[0])

        # check the final position to understand which arm should be used
        arm.data ='left' if bluebox == object[1] else 'right'

        # get the position of the block
        trans = tfBuffer.lookup_transform('world',object[0],rospy.Time())
        # pre-grasp position
        trans.transform.translation.z += 0.15

        # while a possible collision is detected
        collisionDetected = True
        while(collisionDetected):

            # publishing pre-grasp position
            rospy.loginfo("Reaching pre-grasp position")
            response = planner_client(trans,arm)
            trajPub.publish(response.trajectory)
            
            canGo = False
            collisionDetected = False

            # wait unit the target is reached and until a collision is not detected
            while(not canGo and not collisionDetected):
                rospy.sleep(0.2)

            # if a collision is detected wait 2 seconds
            if(collisionDetected):
                rospy.sleep(2)

        # open gripper at pre-grasp position
        isClosed.data = False
        gripperPub.publish(isClosed)

        # grasp position
        trans.transform.translation.z -= 0.15

        # while a possible collision is detected
        collisionDetected = True
        while(collisionDetected):

            # publishing grasp position
            rospy.loginfo("Reaching grasp position")
            response = planner_client(trans,arm)
            trajPub.publish(response.trajectory)

            canGo = False
            collisionDetected = False

            # wait unit the target is reached and until a collision is not detected
            while(not canGo and not collisionDetected):
                rospy.sleep(0.2)

            # if a collision is detected wait 2 seconds
            if(collisionDetected):
                rospy.sleep(2)

        # close gripper at grasp position
        isClosed.data = True
        gripperPub.publish(isClosed)

        # lift-up position
        trans.transform.translation.z += 0.15

        # while a possible collision is detected 
        collisionDetected = True
        while(collisionDetected):

            # publishing lift-up position
            rospy.loginfo("Reaching lift-up position")
            response = planner_client(trans,arm)
            trajPub.publish(response.trajectory)
            
            canGo = False
            collisionDetected = False

            # wait unit the target is reached and until a collision is not detected
            while(not canGo and not collisionDetected):
                rospy.sleep(0.2)

            # if a collision is detected wait 2 seconds
            if(collisionDetected):
                rospy.sleep(2)

        # while a possible collision is detected
        collisionDetected = True
        while(collisionDetected):

            # publishing final position
            rospy.loginfo("Reaching final position\n")
            response = planner_client(object[1],arm)
            trajPub.publish(response.trajectory)
            
            canGo = False
            collisionDetected = False

            # wait unit the target is reached and until a collision is not detected
            while(not canGo and not collisionDetected):
                rospy.sleep(0.2)

            # if a collision is detected wait 2 seconds
            if(collisionDetected):
                rospy.sleep(2)

        # close gripper at grasp position
        isClosed.data = False
        gripperPub.publish(isClosed)

        # go back to the rest position
        rospy.sleep(1)
        restPub.publish(arm)
        rospy.sleep(1)


def baxter_controller():
    """
        Main function that initializes the publishers used in the program, the 
        subscriber to the target reached, tfBuffer in order to read the position
        of the blocks, a client used to plan the trajectories and a service to 
        detect a collision.
    """
    global trajPub, gripperPub, restPub, stopPub, planner_client, tfBuffer

    rospy.init_node('baxter_controller')

    # initializing the publishers
    trajPub = rospy.Publisher('baxter_moveit_trajectory',BaxterTrajectory, queue_size=10)
    gripperPub = rospy.Publisher('baxter_gripper', Bool, queue_size=10)
    restPub = rospy.Publisher('baxter_rest_position', String, queue_size=10)
    stopPub = rospy.Publisher('baxter_stop', Bool, queue_size=10)

    # initialize subscriber
    rospy.Subscriber('baxter_target_reached', Bool, targetReachedCB)

    tfBuffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tfBuffer)

    # wait the baxter_planner_service
    rospy.wait_for_service('baxter_planner_service')
    planner_client = rospy.ServiceProxy('baxter_planner_service',Plan)

    # create the service collision avoidance
    rospy.Service('baxter_collision_avoidance', CollisionAvoidance, collisionService)

    # start the simulation
    rospy.sleep(3)
    startSimulation()

    rospy.spin()

if __name__ == '__main__':
    baxter_controller()
