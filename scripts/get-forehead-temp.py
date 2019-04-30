#!/usr/bin/env python

# ROS integration

import rospy
import tf
import message_filters
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

from std_msgs.msg import Int16MultiArray

from lepton_radiometry.msg import forehead_temp

rospy.init_node('forehead', anonymous=True)
listener = tf.TransformListener()
pubTemp = rospy.Publisher('Forehead_Temp',forehead_temp, queue_size =10)
image_pub = rospy.Publisher('lepton_head',Image, queue_size=10)
bridge = CvBridge()

def ktoc(val):
  return (val - 27315) / 100.0

def getPixels(trans):
	f = 147.3
	z = trans[0]
	x = trans[1]+0.1
	y = trans[2]
	x_pixel = - int(round(f*x/z)) + 80
	y_pixel = - int(round(f*y/z)) + 60
	return x_pixel,y_pixel

def getTemp(x,y,data):
	if y>=0 and y <=120 and x>=0 and x<=160:
		return ktoc(data.data[y*160+x])

def raw_to_8bit(data):
  cv2.normalize(data, data, 0, 65535, cv2.NORM_MINMAX)
  np.right_shift(data, 8, data)
  return cv2.cvtColor(np.uint8(data), cv2.COLOR_GRAY2RGB)

def display_temperature(img, val_k, loc, color):
  # val = ktof(val_k)
  
  val = ktoc(val_k)

  cv2.putText(img,"{0:.1f} degC".format(val), loc, cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
  x, y = loc
  cv2.line(img, (x - 2, y), (x + 2, y), color, 1)
  cv2.line(img, (x, y - 2), (x, y + 2), color, 1)

def callback(data):
	

	datax = data.data
	datax = np.reshape(datax,(120,160))
	datax = datax.astype(np.uint16)
	if datax is None:
		return
	# print datax
	data4x = cv2.resize(datax[:,:], (640, 480))
	minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(data4x)
	img = raw_to_8bit(data4x)
	display_temperature(img, minVal, minLoc, (0, 0, 255))
	display_temperature(img, maxVal, maxLoc, (255, 0, 0))

	try:
		
		rospy.loginfo("callback")
		
		(trans1,rot1) = listener.lookupTransform('/camera_depth_frame', '/head_1', rospy.Time(0))
		x1,y1 = getPixels(trans1)
		print x1,y1
		if y1>=0 and y1 <=120 and x1>=0 and x1<=160:
			print "User 1 Detected!"
			k = data.data[y1*160+x1]			
			# print ktoc(data.data[y1*160+x1])
			meas = forehead_temp()
			meas.user_id = 1
			meas.forehead_temp = ktoc(data.data[y1*160+x1])
			display_temperature(img, k, (4*x1,4*y1), (0, 255, 0))
			cv2.circle(img,(4*x1,4*y1),40,(0, 255, 0),2)
			head2odom = listener.lookupTransform('/odom','head_1',rospy.Time(0))
			meas.x = head2odom[0][0]
			meas.y = head2odom[0][1]
			rospy.loginfo(meas)
			pubTemp.publish(meas)
			
		(trans2,rot2) = listener.lookupTransform('/camera_depth_frame', '/head_2', rospy.Time(0))
		x2,y2 = getPixels(trans2)
		print x2,y2
		if y2>=0 and y2 <=120 and x2>=0 and x2<=160:
			# print ktoc(data.data[y2*160+x2])
			print "User 2 Detected!"
			k = data.data[y2*160+x2]
			meas = forehead_temp()
			meas.user_id = 2
			meas.forehead_temp = ktoc(data.data[y2*160+x2])
			display_temperature(img, k, (4*x2,4*y2), (0, 255, 0))
			cv2.circle(img,(4*x2,4*y2),40,(0, 255, 0),2)
			head2odom = listener.lookupTransform('/odom','/head_2',rospy.Time(0))
			meas.x = head2odom[0][0]
			meas.y = head2odom[0][1]
			rospy.loginfo(meas)
			pubTemp.publish(meas)

		(trans3,rot3) = listener.lookupTransform('/camera_depth_frame', '/head_3', rospy.Time(0))
		x3,y3 = getPixels(trans3)
		print x3,y3
		if y3>=0 and y3 <=120 and x3>=0 and x3<=160:
			# print ktoc(data.data[y2*160+x3])
			print "User 3 Detected!"
			k = data.data[y3*160+x3]
			meas = forehead_temp()
			meas.user_id = 3
			meas.forehead_temp = ktoc(data.data[y3*160+x3])
			display_temperature(img, k, (4*x3,4*y3), (0, 255, 0))
			cv2.circle(img,(4*x3,4*y3),40,(0, 255, 0),2)
			head2odom = listener.lookupTransform('/odom','/head_3',rospy.Time(0))
			meas.x = head2odom[0][0]
			meas.y = head2odom[0][1]
			rospy.loginfo(meas)
			pubTemp.publish(meas)

		(trans4,rot4) = listener.lookupTransform('/camera_depth_frame', '/head_4', rospy.Time(0))
		x4,y4 = getPixels(trans4)
		print x4,y4
		if y4>=0 and y4 <=120 and x4>=0 and x4<=160:
			# print ktoc(data.data[y4*160+x4])
			k = data.data[y4*160+x4]
			meas = forehead_temp()
			meas.user_id = 4
			meas.forehead_temp = ktoc(data.data[y4*160+x4])
			display_temperature(img, k, (4*x4,4*y4), (0, 255, 0))

			head2odom = listener.lookupTransform('/odom','/head_4',rospy.Time(0))
			meas.x = head2odom[0][0]
			meas.y = head2odom[0][1]
			rospy.loginfo(meas)
			pubTemp.publish(meas)

		(trans5,rot5) = listener.lookupTransform('/camera_depth_frame', '/head_5', rospy.Time(0))
		x5,y5 = getPixels(trans5)
		print x5,y5
		if y5>=0 and y5 <=120 and x5>=0 and x5<=160:
			# print ktoc(data.data[y2*160+x5])
			k = data.data[y5*160+x5]
			meas = forehead_temp()
			meas.user_id = 5
			meas.forehead_temp = ktoc(data.data[y5*160+x5])
			display_temperature(img, k, (4*x5,4*y5), (0, 255, 0))

			head2odom = listener.lookupTransform('/odom','/head_5',rospy.Time(0))
			meas.x = head2odom[0][0]
			meas.y = head2odom[0][1]
			rospy.loginfo(meas)
			pubTemp.publish(meas)

		(trans6,rot6) = listener.lookupTransform('/camera_depth_frame', '/head_6', rospy.Time(0))
		x6,y6 = getPixels(trans6)
		print x6,y6
		if y6>=0 and y6 <=120 and x6>=0 and x6<=160:
			# print ktoc(data.data[y2*160+x6])
			k = data.data[y6*160+x6]
			meas = forehead_temp()
			meas.user_id = 6
			meas.forehead_temp = ktoc(data.data[y6*160+x6])
			display_temperature(img, k, (4*x6,4*y6), (0, 255, 0))

			head2odom = listener.lookupTransform('/odom','/head_6',rospy.Time(0))
			meas.x = head2odom[0][0]
			meas.y = head2odom[0][1]
			rospy.loginfo(meas)
			pubTemp.publish(meas)

		(trans7,rot7) = listener.lookupTransform('/camera_depth_frame', '/head_7', rospy.Time(0))
		x7,y7 = getPixels(trans7)
		print x7,y7
		if y7>=0 and y7 <=120 and x7>=0 and x7<=160:
			# print ktoc(data.data[y2*160+x7])
			k = data.data[y7*160+x7]
			meas = forehead_temp()
			meas.user_id = 7
			meas.forehead_temp = ktoc(data.data[y7*160+x7])
			display_temperature(img, k, (4*x7,4*y7), (0, 255, 0))

			head2odom = listener.lookupTransform('/odom','/head_7',rospy.Time(0))
			meas.x = head2odom[0][0]
			meas.y = head2odom[0][1]
			rospy.loginfo(meas)
			pubTemp.publish(meas)
			
	except (tf.ConnectivityException, tf.ExtrapolationException):
		# print "tf exception"
		pass
	except (tf.LookupException):
		print "tf lookup"
		pass

	try:
		image_pub.publish(bridge.cv2_to_imgmsg(img, "rgb8"))
	except CvBridgeError as e:
		print(e)

def getForeheadTemp():
	# print("hey")
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

	# print("what")
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
