import cv2
import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands


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
        for index, landmark in enumerate(hand_landmarks.landmark):
            cx,cy = landmark.x * image_width, landmark.y * image_height
            if index == 0: #手首
                print("index0 = ",end='')
                print(cx,cy)
            if index == 3: #親指第一関節
                print("index3 = ",end='')
                print(cx,cy)
            if index == 7: #人差し指第一関節
                print("index7 = ",end='')
                print(cx,cy)
            if index == 11: #中指第一関節
                print("index11 = ",end='')
                print(cx,cy)
            if index == 15: #薬指第一関節
                print("index15 = ",end='')
                print(cx,cy)
            if index == 19: #小指第一関節
                print("index19 = ",end='')
                print(cx,cy)
        mp_drawing.draw_landmarks(
            image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
    cv2.imshow('MediaPipe Hands', image)
    if cv2.waitKey(5) & 0xFF == 27:
      break
cap.release()