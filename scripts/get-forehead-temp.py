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
listener = tf.TransformListener(False,rospy.Duration(0.1))
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

def get_head_temp(data,img,id):
	id = str(id)
	try:
		now = rospy.Time.now()
		(trans,rot) = listener.lookupTransform('/camera_depth_frame','/head_'+id,rospy.Time(0))
		current = listener.getLatestCommonTime('/camera_depth_frame','/head_'+id)
			
		if now - current < rospy.Duration(0.05):
			x,y = getPixels(trans)
			
			# print x,y
			if y>=0 and y<=120 and x>=0 and x<= 160:
				rospy.loginfo('User '+id+' detected!')
				k = data[y][x]
				
				# Dynamic Sampling Region around head 
				depth = trans[0]
				if abs(depth) > 0:
					radius = int(40.0/abs(depth)) # 40 is head scaling factor
					r = radius/4
					head = data[y-r:y+r,x-r:x+r]
					# Get highest,lowest,average temp around head
					minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(head)
					if head.size !=0:
						average = np.average(head)			
				c = ktoc(maxVal)
				if c>20:	
					meas = forehead_temp()
					meas.user_id = int(id)
					
					meas.high = ktoc(maxVal)
					meas.low = ktoc(minVal)
					meas.average = ktoc(average)

					#Visualization
					display_temperature(img, average, (4*x,4*y),(0,255,0))
					cv2.circle(img,(4*x,4*y),radius,(0,255,0),2)

					#User Location
					head2odom = listener.lookupTransform('/odom','/head_'+id,rospy.Time(0))
					meas.x = head2odom[0][0]
					meas.y = head2odom[0][1]
					
					rospy.loginfo(meas)
					pubTemp.publish(meas)

	except (tf.ConnectivityException, tf.ExtrapolationException):
		# print "tf exception"
		pass
	except (tf.LookupException):
		# print "tf lookup"
		pass

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

	for i in range(1,10):
		get_head_temp(datax,img,i)

	try:
		image_pub.publish(bridge.cv2_to_imgmsg(img, "rgb8"))
	except CvBridgeError as e:
		print(e)

def getForeheadTemp():
	
	lepton_sub = rospy.Subscriber('lepton_raw',Int16MultiArray,callback)
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
