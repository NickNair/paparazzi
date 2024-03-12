#! /usr/bin/python

import cv2
import sys
import argparse
from os import path, getenv
import os

# See the issue and solution here: https://github.com/opencv/opencv/issues/10328
os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'protocol_whitelist;file,rtp,udp'

class VideoFeed:

    def __init__(self, src):

        # Create the video capture device
        self.cap = cv2.VideoCapture(src)
        cv2.namedWindow('rtp')

    def run(self):
        self.running = True

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

        # Show the image in a window
        cv2.imshow('rtp', self.frame)

    def on_key(self, key):
        if key == ord('q'):
            self.running = False

        if key == ord('r'):
            self.rotate = (self.rotate + 1) % 4
            self.mouse['start'] = None



if __name__ == '__main__':

    # change the path here
    filename = "/home/liamligthart/group5/paparazzi/sw/tools/rtp_viewer/rtp_5000.sdp"

    viewer = VideoFeed(filename)

    if not viewer.cap.isOpened():
        raise IOError("Can't open video stream")

    viewer.run()

