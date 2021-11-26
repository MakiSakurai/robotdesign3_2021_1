#!/usr/bin/env python3
import cv2
import mediapipe as mp
import rospy
import math
import sys
from geometry_msgs.msg import Point
from numpy.lib.type_check import imag
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

def main():
  rospy.init_node("unko")
  point_pub = rospy.Publisher("anego", Point,queue_size=10)
  r = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
  # For webcam input:
    cap = cv2.VideoCapture(0)
    with mp_hands.Hands(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as hands:
      while cap.isOpened():
        success, image = cap.read()
        if not success:
          print("Ignoring empty camera frame.")
          # If loading a video, use 'break' instead of 'continue'.
          continue

        # Flip the image horizontally for a later selfie-view display, and convert
        # the BGR image to RGB.
        image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        results = hands.process(image)
        image_height, image_width, _ = image.shape
        # Draw the hand annotations on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if results.multi_hand_landmarks:
          for hand_landmarks in results.multi_hand_landmarks:
            #各指の第一関節の座標を変数に格納
            for index, landmark in enumerate(hand_landmarks.landmark):
                landmark_x = min(int(landmark.x * image_width), image_width - 1)
                landmark_y = min(int(landmark.y * image_height), image_height - 1)

                if index == 3: #親指第一関節
                    #cx3,cy3 = landmark.x * image_width, landmark.y * image_height
                    cx3,cy3 = landmark_x, landmark_y
                if index == 7: #人差し指第一関節
                    cx7,cy7 = landmark_x, landmark_y
                if index == 11: #中指第一関節
                    cx11,cy11 = landmark_x, landmark_y
                if index == 15: #薬指第一関節
                    cx15,cy15 = landmark_x, landmark_y
                if index == 19: #小指第一関節
                    cx19,cy19 = landmark_x, landmark_y
            #取得した座標より各指の中間点を表示
            gap1x,gap1y = (cx3+cx7)/2, (cy3+cy7)/2
            gap1x = int(gap1x)
            gap1y = int(gap1y)
            cv2.circle(image, (gap1x,gap1y), 5, (0, 255, 0), 2)
            gap1x,gap1y = gap1x-image_width/2,-(gap1y-image_height/2)
            
            gap2x,gap2y = (cx7+cx11)/2, (cy7+cy11)/2
            gap2x = int(gap2x)
            gap2y = int(gap2y)
            cv2.circle(image, (gap2x,gap2y), 5, (0, 255, 0), 2)
            gap2x,gap2y = gap2x-image_width/2,-(gap2y-image_height/2)

            gap3x,gap3y = (cx11+cx15)/2, (cy11+cy15)/2
            gap3x = int(gap3x)
            gap3y = int(gap3y)
            cv2.circle(image, (gap3x,gap3y), 5, (0, 255, 0), 2)
            gap3x,gap3y = gap3x-image_width/2,-(gap3y-image_height/2)

            gap4x,gap4y = (cx15+cx19)/2, (cy15+cy19)/2
            gap4x = int(gap4x)
            gap4y = int(gap4y)
            cv2.circle(image, (gap4x,gap4y), 5, (0, 255, 0), 2)
            gap4x,gap4y = gap4x-image_width/2,-(gap4y-image_height/2)

            point = Point()
            point.x = gap1x
            point.y = gap1y
            point.z = 0
            point_pub.publish(point)
            
            cv2.putText(image,"gap1x:"+str(gap1x)+"gap1y:"+str(gap1y),(10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(image,"gap2x:"+str(gap2x)+"gap2y:"+str(gap2y),(10,60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(image,"gap3x:"+str(gap3x)+"gap3y:"+str(gap3y),(10,90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(image,"gap4x:"+str(gap4x)+"gap4y:"+str(gap4y),(10,120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)                                                            

            mp_drawing.draw_landmarks(
                image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
        cv2.imshow('MediaPipe Hands', image)

        r.sleep()
        if cv2.waitKey(5) & 0xFF == 27:
          break
    cap.release()
    
if __name__ == '__main__':
    main()
