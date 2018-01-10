import numpy as np
import cv2

refPt = []			# to store cropping rectangle cordinates
cropping = False	# check point for cropping
frame = []			# to store original image frame

def removeBackground1():
	cap = cv2.VideoCapture(0)
	# cap.set(cv2.CAP_PROP_SETTINGS, 1)
	fgbg = cv2.createBackgroundSubtractorMOG2()

	while(True):
		# Capture frame-by-frame
		ret, frame = cap.read()
		if ret:
			cv2.imshow("Original", frame)
			fgmask = fgbg.apply(frame)
			cv2.imshow("Processed", fgmask)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	# When everything done, release the capture
	cap.release()
	cv2.destroyAllWindows()


def removeBackground2():
	cap = cv2.VideoCapture(0)
	# cap.set(cv2.CAP_PROP_SETTINGS, 1)
	# ret, frame = cap.read()
	# if ret:
	# 	# initial = frame
	# 	initial = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	original = cv2.imread("E:\MachineLearning\Images\TShirt\img70.jpg")

	while(True):
		# Capture frame-by-frame
		ret, frame = cap.read()
		if ret:
			frame = original.copy()			# Process a saved image
			backup = frame.copy()			# An unedited copy of initial image
			# cv2.imshow("Original", backup)
			current = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2GRAY)			# Convert image into grayscale
			median = cv2.medianBlur(current,21)									# Median filtering(second parameter can only be an odd number)
			cv2.imshow("Test1", median)
			thresh = 200
			binary = cv2.threshold(median, thresh, 255, cv2.THRESH_BINARY_INV)[1]			# Convert image into black & white
			# binary = cv2.Canny(median, 30, 150)			# Edge detection(2nd & 3rd parameters are minVal & maxVal, 
															# below min - not edge, above max - sure edge, between - only is connected with sure edge)
			# kernel = np.ones((5,5),np.uint8)
			# binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)	# dilation then erosion
			binary = cv2.dilate(binary, None, iterations=2)			# Dilation - make bigger white area
			binary = cv2.erode(binary, None, iterations=2)			# Erosion - make smaller white area
			cv2.imshow("Test2", binary)
			cnts = cv2.findContours(binary.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]	# Contour tracking()
			cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:2]							# Sort contours area wise from bigger to smaller
			cv2.drawContours(frame, cnts, -1, (0,255,0), 3)				# Draw boundary for contours

			cv2.imshow("Processed", frame)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	# When everything done, release the capture
	cap.release()
	cv2.destroyAllWindows()


def click_and_crop(event, x, y, flags, param):
	global refPt, cropping, frame
	if event == cv2.EVENT_LBUTTONDOWN:			# Capturing mouse click down position
		refPt = [(x,y)]
		cropping = True
	elif event == cv2.EVENT_LBUTTONUP:			# Capturing mouse click up after down position
		refPt.append((x,y))
		cropping = False
		cv2.rectangle(frame, refPt[0], refPt[1], (0, 255, 0), 2)	# Drawing a rectangle on input frame selected by mouse
		cv2.imshow("Original", frame)




def gridMethod():
	cap = cv2.VideoCapture(0)
	# cap.set(cv2.CAP_PROP_SETTINGS, 1)
	global refPt, cropping, frame

	while(True):
		# Capture frame-by-frame
		ret, frame = cap.read()
		if ret:
			if (len(refPt)==2) and (refPt[1][1]-refPt[0][1]>0) and (refPt[1][0]-refPt[0][0]>0):
				cv2.rectangle(frame, refPt[0], refPt[1], (0, 255, 0), 2)	# Drawing a rectangle on input frame selected by mouse
				frame[0:(refPt[1][1]-refPt[0][1]), 0:(refPt[1][0]-refPt[0][0])] = frame[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]
			cv2.namedWindow("Original")

			cv2.imshow("Original", frame)

			# print (frame.shape[1])			# [height, width, noOfColors]
			# print (frame.item(100,100,2))
			# print frame.size()

			# test = frame[350:450, 250:450]
			# frame[200:300, 100:300] = test
			# print (test.item(50,50,0))

			clone = frame.copy()
			# cv2.namedWindow("image")
			cv2.setMouseCallback("Original", click_and_crop)
			if (len(refPt)==2) and (refPt[1][1]-refPt[0][1]>0) and (refPt[1][0]-refPt[0][0]>0):
			# if len(refPt) == 2:
				roi = clone[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]
				cv2.imshow("ROI", roi)
				frame[0:(refPt[1][1]-refPt[0][1]), 0:(refPt[1][0]-refPt[0][0])] = frame[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]

			# cv2.imshow("Processed", frame)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	# When everything done, release the capture
	cap.release()
	cv2.destroyAllWindows()


# removeBackground1()
removeBackground2()
# gridMethod()


'''
class Garment(object):
	def __init__():

	def 


class TShirt(Garment):
'''