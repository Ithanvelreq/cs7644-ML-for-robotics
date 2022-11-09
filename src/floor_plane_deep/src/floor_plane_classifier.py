#!/usr/bin/env python

import rospy

import tensorflow as tf
import numpy as np

from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

class FloorPlaneClassifier:
    def __init__(self):
        self.model_dir_ = rospy.get_param("~model_dir")
        self.ts_ = rospy.get_param("~thumb_size")

        self.load_model()
        
        self.bridge = CvBridge()

        rospy.Subscriber("~image", Image, self.image_callback, queue_size=1)
        self.image_pub_ = rospy.Publisher("~image_label", Image, queue_size=1)

    def load_model(self):
        # Loads the model
        self.model = tf.keras.models.load_model(self.model_dir_)

    def reshape_split(self, image, kernel_size):
        img_height, img_width, channels = image.shape
        tile_heigth, tile_width = kernel_size
        tiled_array = image.reshape(img_height // tile_heigth,
                                    tile_heigth,
                                    img_width // tile_width,
                                    tile_width,
                                    channels)
        tiled_array = tiled_array.swapaxes(1, 2)
        return tiled_array.reshape(-1, self.ts_, self.ts_, 3)

    def image_callback(self, data):
        # Gets the image, check that it is square and divisible by the thumbnail size
        assert(data.height == data.width)
        assert(data.height%self.ts_ == 0)
        # Convert to np array
        img = self.bridge.imgmsg_to_cv2(data,"bgr8")
        # Reshape the image as a batch of thumbnails (faster processing when using batches)
        batch = self.reshape_split(img, (self.ts_, self.ts_))
        # Calls the network
        checked = self.check_thumb(batch)
        # Transforms the array into low resolution image (on pixel per thumbnail)
        low_res = np.reshape(checked,[int(data.height/self.ts_), int(data.width/self.ts_), 3])
        # Upsamples the predictions so they have the same size as the input image
        classified = cv2.resize(low_res,(0,0),fx=self.ts_,fy=self.ts_, interpolation=cv2.INTER_NEAREST).astype(np.uint8)
        overlay = cv2.addWeighted(img, 0.5, classified, 0.5, 0)
        # Publish the result
        enc = self.bridge.cv2_to_imgmsg(overlay,"rgb8")
        self.image_pub_.publish(enc)


    def check_thumb(self, batch):
        # Run the network
        res = self.model(tf.cast(batch, tf.float32), training=False)
        # Makes sure that the output has the proper shape
        assert(res[0].shape[0] == 2)
        answer = np.zeros((res.shape[0], 3))
        predicted = np.argmax(res, axis=1)
        answer[predicted==0, 0] = 255
        answer[predicted==1, 1] = 255
        return answer
           
if __name__ == '__main__':
    rospy.init_node('floor_plane_classifier')
    fp = FloorPlaneClassifier()
    rospy.spin()
