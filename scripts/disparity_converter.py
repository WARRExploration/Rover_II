#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage

pub = rospy.Publisher('disparity_image', Image, queue_size=10)

def callback(data):
    pub.publish(data.image)

def converter():
    
    rospy.init_node('disparity_converter', anonymous=True)

    rospy.Subscriber("disparity", DisparityImage, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    converter()