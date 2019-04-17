#!/usr/bin/env python

# ROS integration

import rospy
import tf
import message_filters
from std_msgs.msg import Int16MultiArray

rospy.init_node('forehead', anonymous=True)
listener = tf.TransformListener()

def ktoc(val):
  return (val - 27315) / 100.0

def getPixels(trans):
	f = 147.3
	z = trans[0]
	x = trans[1]
	y = trans[2]
	x_pixel = - int(round(f*x/z)) + 80
	y_pixel = - int(round(f*y/z)) + 60
	return x_pixel,y_pixel

def callback(data):
	try:
		print("callback")
		# listener = tf.TransformListener()		
		(trans1,rot1) = listener.lookupTransform('/camera_depth_frame', '/head_1', rospy.Time(0))
		x1,y1 = getPixels(trans1)
		print x1,y1
		if y1>=0 and y1 <=120 and x1>=0 and x1<=160:
			print ktoc(data.data[y1*160+x1])
		(trans2,rot2) = listener.lookupTransform('/camera_depth_frame', '/head_2', rospy.Time(0))
		x2,y2 = getPixels(trans2)
		print x2,y2
	except (tf.ConnectivityException, tf.ExtrapolationException):
		pass
	except (tf.LookupException):
		pass

def getForeheadTemp():
	print("hey")
	# rospy.init_node('forehead', anonymous=True)
	# listener = tf.TransformListener()
	# lepton = rospy.Subscriber('lepton_raw',Int16MultiArray,queue_size=1)
	
	lepton_sub = rospy.Subscriber('lepton_raw',Int16MultiArray,callback)

	# while not rospy.is_shutdown():
	# 	if tf.frameExists("/camera_depth_frame") and tf.frameExists("/head_1"):
	# 		t = tf.getLatestCommonTime("/camera_depth_frame", "/head_1")
	# 		position, quaternion = tf.lookupTransform("/camera_depth_frame", "/head_1", t)
	# 		print position, quaternion

	# rate = rospy.Rate(10.0)
	# while not rospy.is_shutdown():
	# 	try:
	# 		(trans1,rot1) = listener.lookupTransform('/camera_depth_frame', '/head_1', rospy.Time(0))
	# 		x1,y1 = getPixels(trans1)
	# 		print x1,y1
	# 		(trans2,rot2) = listener.lookupTransform('/camera_depth_frame', '/head_2', rospy.Time(0))
	# 		x2,y2 = getPixels(trans2)
	# 		print x2,y2
	# 	# except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	# 	except (tf.ConnectivityException, tf.ExtrapolationException):
	# 		continue
	# 	except (tf.LookupException):
	# 		pass

	print("what")
	rospy.spin()

def main():
	try:
		getForeheadTemp()
	except rospy.ROSInterruptException:
		pass	

if __name__ == '__main__':
  	try:
		main()
  	except KeyboardInterrupt:
		print("Shutting down")