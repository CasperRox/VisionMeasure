import numpy as np
import cv2


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
	foreground = initial - current
	cv2.imshow("Test", foreground)
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


removeBackground1()
# removeBackground2()



'''
class Garment(object):
	def __init__():

	def 


class TShirt(Garment):
'''