import cv2
import numpy as np 

#cap = cv2.VideoCapture(0)
image = cv2.imread("img.gif")
lower_green = np.array([45,140,50])
upper_green = np.array([75,255,255])

lower_red = np.array([160,140,50])
upper_red = np.array([180,255,255])

lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])

foundred = False 
#While(True):
	#success,frame = cap.read()
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
hsv = cv2.medianBlur(hsv,5)
imgcircle = cv2.HoughCircles(hsv, cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,mimRadius=10,maxRadius=0)
print imgcircle
#if imgcircle is not None:
#	imgcircle = np.round(imgcircle[0, :].astype("int")
#	for(x, y, r) in imgcircle:
#		print (x,y)
	#rgbIntensity = hsv.at<vec3f>(y,x);
	#float blue = rgbIntensity.val[0];
	#float green = rgbIntensity.val[1];
	#float red = rgbIntensity.val[2];a
cv2.waitKey(0)
