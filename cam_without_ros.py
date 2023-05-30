import cv2 as cv

cam = cv.VideoCapture(0)

while True:
    err, img = cam.read()
    cv.imshow("cv_img", img)
    cv.waitKey(0)
