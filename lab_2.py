#!/usr/bin/env python
 
import rospy, tf, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
# Add additional imports for each of the message types used

#max driving speeds and turning rates
driveSpeed = .25
turnSpeed = 1

#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
	global pose
	global theta
	
	#calculate all the angles and distances for the path
	goal = goal.pose
	q = [goal.orientation.x,goal.orientation.y, goal.orientation.z, goal.orientation.w]
	euler = tf.transformations.euler_from_quaternion(q)
	goalTheta = math.degrees(euler[2])
	globalTheta = math.degrees(math.atan((goal.position.y-pose.position.y)/(goal.position.x-pose.position.x)))
	angleToGoal = globalTheta - math.degrees(theta)	
	distanceToGoal = distanceFrom(goal, pose)
	finalTheta = goalTheta - globalTheta

	#print("start pose: ",pose.position.x, pose.position.y, math.degrees(theta))
	#print("end pose: ", goal.position.x,goal.position.y, goalTheta)
	#print("angleToGoal: ", angleToGoal)
	print "spin!"
	rotate(angleToGoal)
	print "move!"
	driveStraight(driveSpeed, distanceToGoal)
	print "spin!"
	rotate(finalTheta)
	print "done"
	#print(pose.position.x,pose.position.y, math.degrees(theta))


#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
	driveStraight(driveSpeed, .6)
	rotate(90)
	driveStraight(driveSpeed, .45)
	rotate(135)


#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
	if(u1 == u2):
		sendTwist(u1,0)
	timestamp = rospy.get_time()
	angularspeed = (u2-u1)/.2286
	radius = (u1+u2)/(2*(u2-u1))
	vel = angularspeed * radius

	#drive until time has elapsed
	while((rospy.get_time() - timestamp) <= time):
		sendTwist(vel, angularspeed)
	stop()



#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
	startpose = pose

	#drive until distance has been traveled
	while(distanceFrom(startpose, pose) < distance):
		sendTwist(speed, 0)
	stop()

	
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
	global theta
	target = math.degrees(theta)  + angle

	#turn the correct direction
	if(target > 180):
		target = target - 360
	if(target < -180):
		target = target + 360	
	if(angle<0):
		vel = -turnSpeed
	else:
		vel = turnSpeed

	#turn until angle has been reached
	while(math.fabs(math.degrees(theta) - target) > 3):
		sendTwist(0, vel)
	stop()
	



#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
	global theta
	angularspeed = speed/radius
	targetTheta = math.degrees(theta) + angle
	while(math.fabs( math.degrees(theta) - targetTheta) > 3):
		sendTwist(speed, angularspeed)
	stop()


#Bumper Event Callback function
def readBumper(msg):
	if (msg.state == 1):
		executeTrajectory()


# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
	global pose
	pose = Pose()

	(position, orientation) = odom_list.lookupTransform('...','...', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
	
	pass # Delete this 'pass' once implemented

#stops all movement
def stop():
	sendTwist(0,0)


def sendTwist(vel, omega):
	global pub
	msg = Twist()
	msg.linear.x = vel
	msg.angular.z = omega
	pub.publish(msg)


def updateOdom(odomData):
	global pose
	pose = odomData.pose.pose
	q = [pose.orientation.x,pose.orientation.y, pose.orientation.z, pose.orientation.w]
	euler = tf.transformations.euler_from_quaternion(q)
	global theta
	theta = euler[2]

def distanceFrom(startPose, currPose):
	return math.sqrt(math.pow(startPose.position.x - currPose.position.x, 2) + math.pow(startPose.position.y - currPose.position.y, 2))


# This is the program's main function
if __name__ == '__main__':
	# Change this node name to include your username
	rospy.init_node('sample_Lab_2_node')

	# These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
	global pub
	global pose
	global theta
	global odom_tf
	global odom_list

	# Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
	pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) # Publisher for commanding robot motion
	bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
	odom_sub = rospy.Subscriber('/odom', Odometry, updateOdom, queue_size=1) # Callback to update odometry data
	goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, navToPose, queue_size=1) #Callback to handle navigation goal
	# Use this object to get the robot's Odometry 
	odom_list = tf.TransformListener()
	# Use this command to make the program wait for some seconds
	rospy.sleep(rospy.Duration(1, 0))
	print "Starting Lab 2"
	
	#make the robot keep doing something...
	#rospy.Timer(rospy.Duration(...), timerCallback)
	driveArc(.3, .2, 90)
	rospy.sleep(rospy.Duration(30,0))
	# Make the robot do stuff...
	print "Lab 2 complete!"

