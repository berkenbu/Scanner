#!/opt/local/bin/python2.7

import cv2

line_length = 40


def main():
    cv2.namedWindow("Scanner alignment", cv2.CV_WINDOW_AUTOSIZE)
    capture = cv2.VideoCapture(1)
    while True:
        cv2.waitKey(10)
        _,img = capture.read()
        img = cv2.flip(cv2.transpose(img),1)
        height, width, depth = img.shape
        cv2.line(img,(width/2-line_length/2,height/2),(width/2+line_length/2,height/2),(255, 255, 255),1) 
        cv2.line(img,(width/2,height/2-line_length/2),(width/2,height/2+line_length/2),(255, 255, 255),1) 
        cv2.imshow("Scanner alignment", img)

if __name__ == '__main__':
        main()
