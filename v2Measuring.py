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
	initial = cv2.imread("E:\ImageProcessing\Testings\img1.jpg")
	initial = cv2.resize(initial, (640, 480))
	# initial = cv2.cvtColor(initial, cv2.COLOR_BGR2GRAY)
	current = cv2.imread("E:\ImageProcessing\Testings\img2.jpg")
	current = cv2.resize(current, (640, 480))
	# current = cv2.cvtColor(current, cv2.COLOR_BGR2GRAY)
	# foreground = initial - current
	# cv2.imshow("Test", foreground)
	cv2.imshow("Test", current)
	cv2.waitKey(10000)
	# cap = cv2.VideoCapture(0)
	## cap.set(cv2.CAP_PROP_SETTINGS, 1)
	# ret, frame = cap.read()
	# if ret:
	# 	# initial = frame
	# 	initial = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


	# while(True):
	# 	# Capture frame-by-frame
	# 	ret, frame = cap.read()
	# 	if ret:
	# 		cv2.imshow("Original", frame)
	# 		frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	# 		foreground = frame - initial
	# 		cv2.imshow("Processed", foreground)

	# 	if cv2.waitKey(1) & 0xFF == ord('q'):
	# 		break

	# # When everything done, release the capture
	# cap.release()
	# cv2.destroyAllWindows()


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