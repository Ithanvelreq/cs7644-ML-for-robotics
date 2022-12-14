#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

import tensorflow as tf
import numpy as np

class ShoreFollowerDrive:
    def __init__(self):
        self.model_dir_ = rospy.get_param("~model_dir")
        self.linear_vel_ = rospy.get_param('~linear_vel')
        self.twist_factor_ = rospy.get_param('~twist_factor')
        
        self.sess_ = None
        self.load_model()
        
        self.bridge = CvBridge()
        
        rospy.Subscriber("~image", Image, self.image_callback, queue_size=1)
        self.twist_pub_ = rospy.Publisher("~twist", Twist, queue_size=1)

    def load_model(self):
        # Loads the model
        self.model = tf.keras.models.load_model(self.model_dir_)

    def image_callback(self, data):
        # Image call back and resize to proper shape
        # Regularization is done directly inside the model so we don't have to do it.
        raw = self.bridge.imgmsg_to_cv2(data,"bgr8")
        processed_ = np.expand_dims(cv2.resize(raw, (0,0), fx = 32.0/data.height, fy=32.0/data.width, interpolation=cv2.INTER_AREA), axis=0)
        self.twist_pub_.publish(self.image_to_rot(processed_))

    def image_to_rot(self, img):
        # Reads the image, feed it to the network, get the predictions and act on it.
        out = Twist()
        # Runs the network
        res = self.model(tf.cast(img, tf.float32), training=False)[0]
        # Makes sure that the shape of the network matches the required shape
        assert(res.shape[0] == 3)
        print("%5.2f %5.2f %5.2f" %(res[0],res[1],res[2]))
        #TODO: Use the network output so the robot can drive around the lake
        # returns a geometry_msgs.Twist
        res = tf.math.argmax(res)
        if res == 1:
            out.linear.x = 1
        elif res == 0:
            out.linear.x = .5
            out.angular.z = 1
        elif res == 2:
            out.linear.x = .5
            out.angular.z = -1
        return out

if __name__ == '__main__':
        rospy.init_node('shore_follower_drive', anonymous=True)
        fp = ShoreFollowerDrive()
        rospy.spin()

