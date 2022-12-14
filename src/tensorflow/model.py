import tensorflow as tf
from tensorflow.keras import Model
import tensorflow.keras.layers as tfkl

# Made for TensorFlow 2.6.X

# This code implements a Convolutional Neural Network with a tensorboard front-end. Tensorboard
# is a nice visualization tool embedded within TensorFlow.
# Please note that some errors have been intentionaly made inside this code. They can be found in the layers connections or layer size.

class CNNModel(Model):
    # Please note that this model is not build using KERAS, only the keras layers.
    def __init__(self, height, width, channels, classes, mean, std, drop):
        super(CNNModel,self).__init__()
        # REGULARIZATION: Norm using average of each channel and divide by std devition
        self.mean = tf.convert_to_tensor(mean, dtype = tf.float32, name='mean')
        self.std = tf.convert_to_tensor(std, dtype = tf.float32, name='std')
        # MODEL: The following operations define the model of the neural network.
        # DO NOT EDIT, this part is correct.
        #Once the code is working as expected you can modify this part to your liking.
        self.conv1  = tfkl.Conv2D(3, [6,6], strides=(2,2), padding='same', activation=tf.nn.relu, name='conv_1')
        self.norm1  = tfkl.LayerNormalization(name='layer_norm_1')
        self.conv2  = tfkl.Conv2D(6, [5,5], strides=(2,2), padding='same', activation=tf.nn.relu, name='conv_2')
        self.norm2  = tfkl.LayerNormalization(name='layer_norm_2')
        self.conv3  = tfkl.Conv2D(12, [4,4], strides=(2,2), padding='same', activation=tf.nn.relu, name='conv_3')
        self.flat   = tfkl.Flatten()
        self.dense1 = tfkl.Dense(100, name='dense')
        self.drop1  = tfkl.Dropout(drop, name='dropout')
        self.y_     = tfkl.Dense(classes, activation=tf.nn.softmax, name='raw_output')
    
    def call(self, inputs, training=False):
        # errors can be found here
        x = (inputs - self.mean)/self.std
        cv1 = self.conv1(x)
        nm1 = self.norm1(cv1, training=training)
        cv2 = self.conv2(nm1)
        nm2 = self.norm2(cv2, training=training)
        cv3 = self.conv3(nm2)
        fl = self.flat(cv3)
        dn1 = self.dense1(fl)
        dr1 = self.drop1(dn1, training=training)
        dn2 = self.y_(dr1)
        return dn2
