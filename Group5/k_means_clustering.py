#! /usr/bin/python

import cv2
import sys
import argparse
from os import path, getenv
import os
import numpy as np

# See the issue and solution here: https://github.com/opencv/opencv/issues/10328
os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'protocol_whitelist;file,rtp,udp'

class VideoFeed:

    def __init__(self, src):

        # Create the video capture device
        self.cap = cv2.VideoCapture(src)
        self.counter = 0
        cv2.namedWindow('rtp')
        cv2.namedWindow('output')

    def run(self):
        self.running = True
        counter = 0
        # Start an 'infinite' loop
        while self.running:
            # Read a frame from the video capture
            ret, self.frame = self.cap.read()

            # Quit if frame could not be retrieved
            if not ret:
                break

            # Run the computer vision function
            self.cv()

            # Process key input
            self.on_key(cv2.waitKey(1) & 0xFF)

    def cv(self):
        # Rotate the image by increments of 90
        self.frame = cv2.transpose(self.frame)

        self.frame = cv2.flip(self.frame, 0)

        self.k_means()

        # Show the image in a window
        cv2.imshow('rtp', self.frame)
        cv2.imwrite("data/data_{}.png".format(self.counter), self.frame)
        self.counter+=1

    def on_key(self, key):
        if key == ord('q'):
            self.running = False

        if key == ord('r'):
            self.rotate = (self.rotate + 1) % 4
            self.mouse['start'] = None
    
    def k_means(self):
        # Reshaping the image into a 2D array of pixels and 3 color values (RGB)
        pixel_vals = self.frame.reshape((-1,3))

        # Convert to float type
        pixel_vals = np.float32(pixel_vals)

        #the below line of code defines the criteria for the algorithm to stop running, 
        #which will happen is 100 iterations are run or the epsilon (which is the required accuracy) 
        #becomes 85%
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.85)

        # then perform k-means clustering with number of clusters defined as 3
        #also random centres are initially choosed for k-means clustering
        k = 3
        retval, labels, centers = cv2.kmeans(pixel_vals, k, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

        # convert data into 8-bit values
        centers = np.uint8(centers)
        segmented_data = centers[labels.flatten()]

        # reshape data into the original image dimensions
        segmented_image = segmented_data.reshape((self.frame.shape))

        cv2.imshow('output', segmented_image)

        


if __name__ == '__main__':

    # change the path here
    filename = "/home/nicknair/AE4317/paparazzi/sw/tools/rtp_viewer/rtp_5000.sdp"

    viewer = VideoFeed(filename)

    if not viewer.cap.isOpened():
        raise IOError("Can't open video stream")

    viewer.run()

