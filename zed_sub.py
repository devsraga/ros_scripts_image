#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2

MARGIN = 10  # pixels
FONT_SIZE = 1
FONT_THICKNESS = 1
HANDEDNESS_TEXT_COLOR = (88, 205, 54) # vibrant green

def draw_landmarks_on_image(rgb_image, detection_result):
  hand_landmarks_list = detection_result.hand_landmarks
  handedness_list = detection_result.handedness
  annotated_image = np.copy(rgb_image)

  # Loop through the detected hands to visualize.
  for idx in range(len(hand_landmarks_list)):
    hand_landmarks = hand_landmarks_list[idx]
    handedness = handedness_list[idx]

    # Draw the hand landmarks.
    hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
    hand_landmarks_proto.landmark.extend([landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in hand_landmarks])
    solutions.drawing_utils.draw_landmarks(
      annotated_image,
      hand_landmarks_proto,
      solutions.hands.HAND_CONNECTIONS,
      solutions.drawing_styles.get_default_hand_landmarks_style(),
      solutions.drawing_styles.get_default_hand_connections_style())

    # Get the top left corner of the detected hand's bounding box.
    height, width, _ = annotated_image.shape
    x_coordinates = [landmark.x for landmark in hand_landmarks]
    y_coordinates = [landmark.y for landmark in hand_landmarks]
    text_x = int(min(x_coordinates) * width)
    text_y = int(min(y_coordinates) * height) - MARGIN

    # Draw handedness (left or right hand) on the image.
    cv.putText(annotated_image, f"{handedness[0].category_name}",
                (text_x, text_y), cv.FONT_HERSHEY_DUPLEX,
                FONT_SIZE, HANDEDNESS_TEXT_COLOR, FONT_THICKNESS, cv.LINE_AA)

  return annotated_image

def cam_call(data):
    # rospy.loginfo("The receved data is: ")
    # print(data.header)
    # print(type(data.header))
    # print(data.height)
    # print(data.width)
    # print(np.shape(data.data))
    # print(data.encoding)
    # print(data.is_bigendian)
    # print(data.step)
    # print(type(data.data))


    img = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

    # STEP 2: Create an HandLandmarker object.
    base_options = python.BaseOptions(model_asset_path='/home/pandu_cpri/dev_ws/src/dev_pack/ros_scripts_image/hand_landmarker.task')
    options = vision.HandLandmarkerOptions(base_options=base_options,num_hands=2)
    detector = vision.HandLandmarker.create_from_options(options)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=img)
    # STEP 4: Detect hand landmarks from the input image.
    detection_result = detector.detect(mp_image)

    # STEP 5: Process the classification result. In this case, visualize it.
    annotated_image = draw_landmarks_on_image(mp_image.numpy_view(), detection_result)
    # cv.imshow("cam_img", img)
    cv.imshow("mp",annotated_image)
        # Press Q on keyboard to exit
    if cv.waitKey(1) & 0xFF == ord('q'):
        cv.destroyAllWindows()
def main():
    rospy.init_node("cam_sub", anonymous=True)
    rospy.loginfo("cam_sub node is initialised")
    rospy.Subscriber('/zed2/zed_node/rgb_raw/image_raw_color', Image, cam_call)
    rate = rospy.Rate(3)
    rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    bridge = CvBridge()
    main()

