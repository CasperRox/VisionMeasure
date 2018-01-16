import numpy as np
import cv2
import math

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
	# (height,width) = original.shape[:2]
	# rotation_matrix1 = cv2.getRotationMatrix2D((width/2, height/2), 180, 1)			# Rotation matrix ((centerOfRotation), Anti-ClockwiseRotationAngle, Scale)
	# original = cv2.warpAffine(original, rotation_matrix1, (width,height))				# Rotate filtered image (Image, RotationMatrix, NewImageDimensions)

	while(True):
		# Capture frame-by-frame
		ret, frame = cap.read()
		if ret:
			# print("New frame")
			frame = original.copy()			# Process a saved image instead of live video
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
			# cv2.ellipse(frame, ellipse, (0,0,255), 3)
			print(int(ellipse[0][0]), int(ellipse[0][1]))

			box = cv2.boxPoints(ellipse)			# Take 4 cordinates of enclosing rectangle for the ellipse
			box = np.int0(box)
			# cv2.drawContours(frame,[box],0,(0,0,255),3)
			# print(box)

			frame_diagonal = int(math.sqrt(math.pow(frame.shape[0],2) + math.pow(frame.shape[1],2)))
			rotation_matrix = cv2.getRotationMatrix2D(ellipse[0], (ellipse[2]-90), 1)			# Rotation matrix ((centerOfRotation), Anti-ClockwiseRotationAngle, Scale)
			# rotation_matrix = cv2.getRotationMatrix2D((int(ellipse[0][0]),int(ellipse[0][1])), (int(ellipse[2])-90), 1)
			rotated_mask = cv2.warpAffine(mask, rotation_matrix, (mask.shape[1],mask.shape[0]))				# Rotate filtered image (Image, RotationMatrix, NewImageDimensions)
			# rotated_mask = cv2.warpAffine(mask, rotation_matrix, (640, 640))				# Rotate filtered image (Image, RotationMatrix, NewImageDimensions)
			rotated_frame = cv2.warpAffine(frame, rotation_matrix, (frame_diagonal,frame_diagonal))				# Rotate actual image
			# rotated_frame = cv2.warpAffine(frame, rotation_matrix, (frame.shape[1],frame.shape[0]))				# Rotate actual image
			# rotated_frame = cv2.warpAffine(frame, rotation_matrix, (640, 640))				# Rotate actual image
			cv2.imshow("Test4", rotated_mask)

			# *************************************************************
			# *************************************************************
			# ***********************Body Height**************************************
			# *************************************************************
			# *************************************************************
			height_array_y = int(ellipse[0][1])
			# print(rotated_mask[height_array_y])
			white = False
			first = 0
			last = 0
			for i in range(0,len(rotated_mask[height_array_y])):				# Calculate pixel height by checking pixel value
				if white == False and rotated_mask[height_array_y][i] != 0:
					first = i
					white = True
				elif white == True and rotated_mask[height_array_y][i] == 0:
					last = i-1
					white = False
			pixel_height = last - first
			print("pixelHeight = %d" %pixel_height)
			cv2.line(rotated_frame, (first,height_array_y), (last,height_array_y), (255,0,0), 3)			# Draw height calculating line on image
			font = cv2.FONT_HERSHEY_SCRIPT_COMPLEX
			cv2.putText(rotated_frame, '%.1f pixel' %pixel_height, (first,height_array_y-10), font, 1, (255,0,0), 2, cv2.LINE_AA)			# Display height value on image



			mid_width_array_x = int(ellipse[0][0])
			sleeve_check_length = int(pixel_height * 27 /100)
			# cv2.line(rotated_frame, (mid_width_array_x,0), (mid_width_array_x,480), (255,0,0), 3)			# Draw height calculating line on image
			# cv2.line(rotated_frame, (mid_width_array_x-sleeve_check_length,0), (mid_width_array_x-sleeve_check_length,480), (255,255,0), 3)			# Draw height calculating line on image
			# cv2.line(rotated_frame, (mid_width_array_x+sleeve_check_length,0), (mid_width_array_x+sleeve_check_length,480), (255,255,0), 3)			# Draw height calculating line on image


			# *************************************************************
			# *************************************************************
			# ***********************Body Sweap**************************************
			# *************************************************************
			# *************************************************************
			transpose_rotated_mask = np.transpose(rotated_mask)
			sleeve_check_temp1_count = np.count_nonzero(transpose_rotated_mask[mid_width_array_x-sleeve_check_length])
			sleeve_check_temp2_count = np.count_nonzero(transpose_rotated_mask[mid_width_array_x+sleeve_check_length])
			sleeve_side = 0
			non_sleeve_side = 0
			rotated = False
			if sleeve_check_temp1_count > sleeve_check_temp2_count:
				sleeve_side = mid_width_array_x-sleeve_check_length
				non_sleeve_side = mid_width_array_x+sleeve_check_length
				rotated = False
			else:
				sleeve_side = mid_width_array_x+sleeve_check_length
				non_sleeve_side = mid_width_array_x-sleeve_check_length
				rotated = True





			body_sweap_x = 0
			temp_width_pre = np.count_nonzero(transpose_rotated_mask[non_sleeve_side])
			step = 5
			while True:
				if rotated == False:
					non_sleeve_side += step
					temp_count = np.count_nonzero(transpose_rotated_mask[non_sleeve_side])
					if temp_count < temp_width_pre:
						temp_count2 = np.count_nonzero(transpose_rotated_mask[non_sleeve_side+step])
						if temp_count2 < temp_count:
							body_sweap_x = non_sleeve_side-step
							break
					else:
						temp_width_pre = temp_count
				else:
					non_sleeve_side -= step
					temp_count = np.count_nonzero(transpose_rotated_mask[non_sleeve_side])
					if temp_count < temp_width_pre:
						temp_count2 = np.count_nonzero(transpose_rotated_mask[non_sleeve_side-step])
						if temp_count2 < temp_count:
							body_sweap_x = non_sleeve_side+step
							break
					else:
						temp_width_pre = temp_count
			white = False
			first = 0
			last = 0
			for i in range(0,len(transpose_rotated_mask[body_sweap_x])):				# Calculate pixel body sweap by checking pixel value
				if white == False and transpose_rotated_mask[body_sweap_x][i] != 0:
					first = i
					white = True
				elif white == True and transpose_rotated_mask[body_sweap_x][i] == 0:
					last = i-1
					white = False
			pixel_body_sweap = last - first
			print("pixelBodySweap = %d" %pixel_body_sweap)
			cv2.line(rotated_frame, (body_sweap_x,first), (body_sweap_x,last), (255,0,0), 3)			# Draw height calculating line on image
			font = cv2.FONT_HERSHEY_SCRIPT_COMPLEX
			cv2.putText(rotated_frame, '%.1f pixel' %pixel_body_sweap, (body_sweap_x-100,first-10), font, 1, (255,0,0), 2, cv2.LINE_AA)			# Display height value on image




			# *************************************************************
			# *************************************************************
			# *************************Body Width************************************
			# *************************************************************
			# *************************************************************
			body_width_x = 0
			temp_width_pre = np.count_nonzero(transpose_rotated_mask[mid_width_array_x])
			sleeve_check = mid_width_array_x
			step = 5
			body_width_x_dif = int(33/step)
			body_width_first = []
			body_width_last = []
			count_for_dif = 0
			dif = 5
			continuous_white_counts = []
			max_index = 0
			first = []
			last = []
			while True:
				if rotated == False:
					count_for_dif += 1
					sleeve_check -= step
					white = False
					continuous_white = 0
					continuous_white_counts = []
					for i in range(0,len(transpose_rotated_mask[sleeve_check])):				# Calculate cintinuous white by checking pixel value
						if white == False and transpose_rotated_mask[sleeve_check][i] != 0:
							first.append(i)
							continuous_white = 1
							# continuous_white += 1
							white = True
						elif white == True and transpose_rotated_mask[sleeve_check][i] != 0:
							continuous_white += 1
							white = True
						elif white == True and transpose_rotated_mask[sleeve_check][i] == 0:
							continuous_white_counts.append(continuous_white)
							print("list %d" %i)
							print("white %d" %continuous_white)
							last.append(i)
							white = False
					# continuous_white_counts = sorted(continuous_white_counts, reverse=True)
					# cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:1]							# Sort contours area wise from bigger to smaller
					print("list size %d" %len(continuous_white_counts))
					max_index = np.argmax(continuous_white_counts)
					print("max %d" %max_index)
					if count_for_dif - body_width_x_dif > 0:
						body_width_first.append(first[max_index])
						body_width_last.append(last[max_index])
						print("for actual width %d" %len(body_width_first))
					temp_count = continuous_white_counts[max_index]
					print("***** count %d" %temp_count)
					print("pre %d" %temp_width_pre)
					if temp_count > temp_width_pre+dif:
						body_width_x = sleeve_check+step
						break
					# if temp_count < temp_width_pre:
					# 	temp_count2 = np.count_nonzero(transpose_rotated_mask[non_sleeve_side+step])
					# 	if temp_count2 < temp_count:
					# 		body_sweap_x = non_sleeve_side-step
					# 		break
					else:
						temp_width_pre = temp_count
				else:
					count_for_dif += 1
					sleeve_check += step
					white = False
					continuous_white = 0
					continuous_white_counts = []
					for i in range(0,len(transpose_rotated_mask[sleeve_check])):				# Calculate cintinuous white by checking pixel value
						if white == False and transpose_rotated_mask[sleeve_check][i] != 0:
							first.append(i)
							continuous_white = 1
							# continuous_white += 1
							white = True
						elif white == True and transpose_rotated_mask[sleeve_check][i] != 0:
							continuous_white += 1
							white = True
						elif white == True and transpose_rotated_mask[sleeve_check][i] == 0:
							continuous_white_counts.append(continuous_white)
							last.append(i)
							white = False
					# continuous_white_counts = sorted(continuous_white_counts, reverse=True)
					# cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:1]							# Sort contours area wise from bigger to smaller
					max_index = np.argmax(continuous_white_counts)
					if count_for_dif - body_width_x_dif > 0:
						body_width_first.append(first[max_index])
						body_width_last.append(last[max_index])
					temp_count = continuous_white_counts[max_index]
					if temp_count > temp_width_pre+dif:
						body_width_x = sleeve_check-step
						break
					else:
						temp_width_pre = temp_count
			pixel_body_width = continuous_white_counts[max_index]
			pixel_body_width2 = last[max_index] - first[max_index]
			# white = False
			# first = 0
			# last = 0
			# for i in range(0,len(transpose_rotated_mask[body_sweap_x])):				# Calculate pixel body sweap by checking pixel value
			# 	if white == False and transpose_rotated_mask[body_sweap_x][i] != 0:
			# 		first = i
			# 		white = True
			# 	elif white == True and transpose_rotated_mask[body_sweap_x][i] == 0:
			# 		last = i-1
			# 		white = False
			# pixel_body_sweap = last - first

			# print("pixelBodyWidth = %d" %pixel_body_width)
			# cv2.line(rotated_frame, (body_width_x,first[max_index]), (body_width_x,last[max_index]), (255,0,0), 3)			# Draw height calculating line on image
			# font = cv2.FONT_HERSHEY_SCRIPT_COMPLEX
			# cv2.putText(rotated_frame, '%.1f pixel' %pixel_body_width, (body_width_x-100,first[max_index]-10), font, 1, (255,0,0), 2, cv2.LINE_AA)			# Display height value on image

			pixel_body_width_actual = body_width_last[len(body_width_last)-1] - body_width_first[len(body_width_first)-1]
			print("pixelBodyWidthActual = %d" %pixel_body_width_actual)
			if rotated == False:
				cv2.line(rotated_frame, ((body_width_x + body_width_x_dif),body_width_first[len(body_width_first)-1]), ((body_width_x + body_width_x_dif),body_width_last[len(body_width_last)-1]), (255,0,0), 3)			# Draw height calculating line on image
			else:
				cv2.line(rotated_frame, ((body_width_x - body_width_x_dif),body_width_first[len(body_width_first)-1]), ((body_width_x - body_width_x_dif),body_width_last[len(body_width_last)-1]), (255,0,0), 3)			# Draw height calculating line on image
			font = cv2.FONT_HERSHEY_SCRIPT_COMPLEX
			cv2.putText(rotated_frame, '%.1f pixel' %pixel_body_width_actual, (body_width_x-150,body_width_first[len(body_width_first)-1]-10), font, 1, (255,0,0), 2, cv2.LINE_AA)			# Display height value on image




			print(np.count_nonzero(rotated_mask[height_array_y]))
			print(np.count_nonzero(np.transpose(rotated_mask)[mid_width_array_x-sleeve_check_length]))
			print(np.count_nonzero(np.transpose(rotated_mask)[mid_width_array_x+sleeve_check_length]))

			rotation_matrix = cv2.getRotationMatrix2D(ellipse[0], (360-(ellipse[2]-90)), 1)			# Rotation matrix ((centerOfRotation), Anti-ClockwiseRotationAngle, Scale)
			rotated_frame = cv2.warpAffine(rotated_frame, rotation_matrix, (frame.shape[1],frame.shape[0]))				# Rotate actual image

			rotated_frame = cv2.resize(rotated_frame, (960,720))
			cv2.imshow("Test5", rotated_frame)

			# cv2.imshow("Processed", frame)

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