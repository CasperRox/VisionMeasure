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
	original = cv2.imread("E:\MachineLearning\Images\TShirt\img2890.jpg")

	while(True):
		# Capture frame-by-frame
		ret, frame = cap.read()
		if ret:
			# print("New frame")
			# frame = original.copy()			# Process a saved image instead of live video
			backup = frame.copy()			# An unedited copy of initial image
			# cv2.imshow("Original", backup)
			gray = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2GRAY)				# Convert image into grayscale
			median = cv2.medianBlur(gray, 11)									# Median filtering(second parameter can only be an odd number)
			# cv2.imshow("Test1", median)
			thresh = 200
			binary = cv2.threshold(median, thresh, 255, cv2.THRESH_BINARY_INV)[1]			# Convert image into black & white
			# binary = cv2.Canny(median, 30, 150)			# Edge detection(2nd & 3rd parameters are minVal & maxVal, 
															# below min -> not edge, above max -> sure edge, between -> only is connected with sure edge)
			# kernel = np.ones((5,5),np.uint8)
			# binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)	# dilation then erosion
			binary = cv2.dilate(binary, None, iterations=2)			# Dilation - make bigger white area
			binary = cv2.erode(binary, None, iterations=2)			# Erosion - make smaller white area
			cv2.imshow("Test2", binary)
			cnts = cv2.findContours(binary.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]	# Contour tracking(), function will modify source image
			cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:1]							# Sort contours area wise from bigger to smaller
			# print(len(cnts[0]))
			cv2.drawContours(frame, cnts, -1, (0,255,0), 3)				# Draw boundary for contour(-1 -> draw all contours, 3 -> width of boundary)

			mask = np.zeros(gray.shape,np.uint8)				# Create a black colored empty frame
			cv2.drawContours(mask, cnts, 0, 255, -1)			# Draw T.shirt on it (0 -> contourIndex, 255 -> color(white), -1 -> filledContour)
																# Color can be represented using one integer since "mask" is black & white (or grayscale)
			cv2.imshow("Test3", mask)

			ellipse = cv2.fitEllipse(cnts[0])				# [ellipse] = [(center), (MajorAxisLength, MinorAxisLength), clockwiseAngleFromXAxisToMajorOrMinorAxis]
			cv2.ellipse(frame, ellipse, (0,0,255), 3)
			print(int(ellipse[0][0]), int(ellipse[0][1]))

			box = cv2.boxPoints(ellipse)			# Take 4 cordinates of enclosing rectangle for the ellipse
			box = np.int0(box)
			cv2.drawContours(frame,[box],0,(0,0,255),3)
			# print(box)

			rotation_matrix = cv2.getRotationMatrix2D(ellipse[0], (ellipse[2]-90), 1)			# Rotation matrix ((centerOfRotation), Anti-ClockwiseRotationAngle, Scale)
			# rotation_matrix = cv2.getRotationMatrix2D((int(ellipse[0][0]),int(ellipse[0][1])), (int(ellipse[2])-90), 1)
			rotated_mask = cv2.warpAffine(mask, rotation_matrix, (640, 640))				# Rotate filtered image (Image, RotationMatrix, NewImageDimensions)
			rotated_frame = cv2.warpAffine(frame, rotation_matrix, (640, 640))				# Rotate actual image
			cv2.imshow("Test4", rotated_mask)

			height_array_y = int(ellipse[0][1])
			# print(rotated_mask[height_array_y])
			white = False
			first = 0
			last = 0
			for i in range(0,len(rotated_mask[height_array_y])):
				if white == False and rotated_mask[height_array_y][i] != 0:
					first = i
					white = True
				elif white == True and rotated_mask[height_array_y][i] == 0:
					last = i-1
					white = False
			pixel_height = last - first
			print(pixel_height)
			cv2.line(rotated_frame, (first,height_array_y), (last,height_array_y), (255,0,0), 3)
			font = cv2.FONT_HERSHEY_SCRIPT_COMPLEX
			cv2.putText(rotated_frame, '%.1f pixel' %pixel_height, (10,80), font, 1, (255,0,0), 2, cv2.LINE_AA)
			cv2.imshow("Test5", rotated_frame)

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
			frame = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2GRAY)				# Convert image into grayscale
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