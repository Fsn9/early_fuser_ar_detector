#!/usr/bin/env python

import cv2
import rospy
import rospkg
import os
from shapely.geometry import Polygon

AREA_THRESHOLD = 1000 # area in pixel coordinates to be considered a marker

def labeller():
	rospy.init_node('aruco_labeller', anonymous=True)

	# Get package path
	rospack = rospkg.RosPack()
	pkg_path = rospack.get_path('platform_detector')

	# Get dataset to label param defined in the launch file
	dataset_dir_path = rospy.get_param("~dataset_to_label_folder")
	
	# Get dataset dir path given by the user
	dataset_path = os.path.join(pkg_path, "datasets", dataset_dir_path)
	images_files = os.listdir(dataset_path)

	# Define aruco
	aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
	aruco_params = cv2.aruco.DetectorParameters_create()

	# Define dictionary from image to detected corner, ids and rejected
	image2detection_data = {}
	for image_file in images_files:
		max_area = -10000
		max_idx = -1
		max_corners = []
		max_ids = []

		# Read image
		image = cv2.imread(os.path.join(dataset_path, image_file))
		(corners, ids, rejected) = cv2.aruco.detectMarkers(image, aruco_dict, parameters=aruco_params)

		# verify *at least* one ArUco marker was detected
		if len(corners) > 0:
			# flatten the ArUco IDs list
			ids = ids.flatten()
			# loop over the detected ArUCo corners
			areas = []
			for idx, (markerCorner, markerID) in enumerate(zip(corners, ids)):
				# extract the marker corners (which are always returned in
				# top-left, top-right, bottom-right, and bottom-left order)
				corners = markerCorner.reshape((4, 2))
				(topLeft, topRight, bottomRight, bottomLeft) = corners
				# convert each of the (x, y)-coordinate pairs to integers
				topRight = (int(topRight[0]), int(topRight[1]))
				bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
				bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
				topLeft = (int(topLeft[0]), int(topLeft[1]))
				# Compute area
				areas.append(Polygon((topRight, bottomRight, bottomLeft, topLeft)).area)
				if len(corners) == 1 and areas[-1] > AREA_THRESHOLD:
					image2detection_data[image_file] = {'image': image, 'corners': corners, 'ids': ids}
					break		

				if areas[-1] > AREA_THRESHOLD and areas[-1] > max_area:
					max_area = areas[-1]
					max_idx = idx
					max_corners = corners
					max_ids = ids
				if len(max_corners) > 0:
					image2detection_data[image_file] = {'image': image, 'corners': max_corners, 'ids': ids[max_idx]}

	for image_file in image2detection_data.keys():
		print(f'[INFO] {image_file} labelled')
		# Get corners, ids and image
		markerCorner = image2detection_data[image_file]["corners"]
		markerID = image2detection_data[image_file]["ids"]
		image = image2detection_data[image_file]["image"]

		# Prepare data to show
		corners = markerCorner.reshape((4, 2))
		(topLeft, topRight, bottomRight, bottomLeft) = corners
		topRight = (int(topRight[0]), int(topRight[1]))
		bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
		bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
		topLeft = (int(topLeft[0]), int(topLeft[1]))
		cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
		cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
		cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
		cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
		cX = int((topLeft[0] + bottomRight[0]) / 2.0)
		cY = int((topLeft[1] + bottomRight[1]) / 2.0)
		cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
		cv2.putText(image, str(markerID),
			(topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
			0.5, (0, 255, 0), 2)
		# Show labelled image
		#cv2.imshow("Image", image)
		#cv2.waitKey(0)

if __name__ == '__main__':
    try:
        labeller()
    except rospy.ROSInterruptException:
        pass