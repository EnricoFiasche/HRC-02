#!/usr/bin/env python

import sys
import math
import rospy
import tf2_ros

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from human_baxter_collaboration.srv import CollisionAvoidance

## transform buffer used to read the position sent by the tf adapter 
tfBuffer = None

## Bool value true used to send a "collision detected" message to the server
true = None

## client collision avoidance
collision_avoidance_client = None

def leftGripperCB(gripperPos):
	"""
		Callback function that reads the position of the left gripper sent by unity.

		@param:
			- gripperPos: PoseStamped variable, which contains left gripper position 
	"""
	global tfBuffer, collision_avoidance_client, true

	# take human hands positions
	left_human_hand = tfBuffer.lookup_transform('world','hand_l',rospy.Time())
	right_human_hand = tfBuffer.lookup_transform('world','hand_r',rospy.Time())
	left_human_arm = tfBuffer.lookup_transform('world','lowerarm_l',rospy.Time())
	right_human_arm = tfBuffer.lookup_transform('world','lowerarm_r',rospy.Time())

    # evaluate the distance and check a possible collision
	if(checkDistance(gripperPos.pose.position, left_human_hand.transform.translation, right_human_hand.transform.translation)):
		collision_avoidance_client(true) # collision detected
	elif(checkDistance(gripperPos.pose.position, left_human_arm.transform.translation, right_human_arm.transform.translation)):
		collision_avoidance_client(true) # collision detected

def righGripperCB(gripperPos):
	"""
		Callback function that reads the position of the right gripper sent by unity.

		@param:
			- gripperPos: PoseStamped variable, which contains right gripper position 
	"""
	global tfBuffer, collision_avoidance_client, true

	# take human hands and arms positions
	left_human_hand = tfBuffer.lookup_transform('world','hand_l',rospy.Time())
	right_human_hand = tfBuffer.lookup_transform('world','hand_r',rospy.Time())
	left_human_arm = tfBuffer.lookup_transform('world','lowerarm_l',rospy.Time())
	right_human_arm = tfBuffer.lookup_transform('world','lowerarm_r',rospy.Time())

    # evaluate the distance and check a possible collision
	if(checkDistance(gripperPos.pose.position, left_human_hand.transform.translation, right_human_hand.transform.translation)):
		collision_avoidance_client(true) # collision detected
	elif(checkDistance(gripperPos.pose.position, left_human_arm.transform.translation, right_human_arm.transform.translation)):
		collision_avoidance_client(true) # collision detected


def checkDistance(position, left_human, right_human):
	"""
		Fuction used to evaluate the distance between the gripper of Baxter and the
		human hands. If the distance is less than 10 cm a possible collision is detected.

		@param:
			- position: baxter gripper position
			- left_human: human left hand position
			- right_human: human right hand position

		@return:
			- True: if a possible collision is detected, so if the distance
					between the gripper and the hand is less than 10 cm
			- False: if there is no collision detected.
	"""

	# evaluate the distance between the gripper and the human hands
	leftDistance = math.dist([position.x, position.y, position.z], [left_human.x, left_human.y, left_human.z])
	rightDistance = math.dist([position.x, position.y, position.z], [right_human.x, right_human.y, right_human.z])

	# if a possible collision is detected, a true value is returned
	if(leftDistance < 0.10):
		return True
	elif(rightDistance < 0.10):
		return True

	return False

def main():
	"""
		Main function that initializes the tfBuffer in order to read the position
		of the human, sets the subscriber to the grippers position and create a 
		client to advise if a collision is detected.
	"""
	global tfBuffer, collision_avoidance_client, true

	rospy.init_node('baxter_collision_avoidance')

	# initializing tfBuffer
	tfBuffer = tf2_ros.Buffer()
	tf2_ros.TransformListener(tfBuffer)

	# subscribers to the grippers position
	rospy.Subscriber('baxter_left_gripper', PoseStamped, leftGripperCB)
	rospy.Subscriber('baxter_right_gripper', PoseStamped, righGripperCB)

	# wait until the service collision avoidance is active, then initialize a client to this service
	rospy.wait_for_service('baxter_collision_avoidance')
	collision_avoidance_client = rospy.ServiceProxy('baxter_collision_avoidance', CollisionAvoidance)

	# initializing true value
	true = Bool()
	true.data = True

	rospy.sleep(4)
	rospy.spin()

if __name__ == "__main__":
    main()