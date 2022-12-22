#!/usr/bin/env python

import cv2
import rospy
import rospkg
import os
from shapely.geometry import Polygon
from sklearn.model_selection import train_test_split
import shutil
import yaml

AREA_THRESHOLD = 1000 # area in pixel coordinates to be considered a marker
IMG_HEIGHT = 0
IMG_WIDTH = 0

def touch_data_yaml_file(dataset_path):
	content = """train: ../train/images\nval: ../valid/images\ntest: ../test/images\n\nnc: 1\nnames: ['artuga']"""
	#content = yaml.safe_load(content)
	
	with open(os.path.join(dataset_path, "data.yaml"), 'w') as yaml_file:
		#yaml.dump(content, yaml_file, default_flow_style=False, sort_keys=False)
		yaml_file.write(content)

def make_dataset():
	rospy.init_node('dataset_maker', anonymous=True)

	# Get package path
	rospack = rospkg.RosPack()
	pkg_path = rospack.get_path('platform_detector')

	# Get dataset to label param defined in the launch file
	dataset_dir_path = rospy.get_param("dataset_to_label_folder")
	
	# Get dataset dir path given by the user
	dataset_path = os.path.join(pkg_path, "datasets", dataset_dir_path)
	images_files = os.listdir(dataset_path)

	# Get train, test, valid ratios
	test_ratio = rospy.get_param("test_ratio")
	valid_ratio = rospy.get_param("valid_ratio")
	train_ratio = 1 - (valid_ratio + test_ratio)
	images_train, images_rem, _, _ = train_test_split(images_files, [x for x in range(len(images_files))], train_size = train_ratio)
	images_valid, images_test, _, _ = train_test_split(images_rem, [x for x in range(len(images_rem))], test_size = test_ratio / (test_ratio + valid_ratio))
	
	# Create directories
	train_dir = os.path.join(dataset_path, "train")
	test_dir = os.path.join(dataset_path, "test")
	valid_dir = os.path.join(dataset_path, "valid")

	dirs = [train_dir, test_dir, valid_dir]
	for dir in dirs:
		try: os.mkdir(dir)
		except OSError as error: print(error);
		try: os.mkdir(os.path.join(dir, "images"))
		except OSError as error: print(error);
		try: os.mkdir(os.path.join(dir, "labels"))
		except OSError as error: print(error);
	
	# Touch data.yaml file
	touch_data_yaml_file(dataset_path)

	""" ARUCO detection"""
	# Define aruco
	aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
	aruco_params = cv2.aruco.DetectorParameters_create()

	# Define flag for getting image size
	first_image = True

	# Define dictionary from image to detected corner, ids and rejected
	image2detection_data = {}
	for image_file in images_files:
		# Save image in new dir structure
		if image_file in images_train: shutil.copyfile(os.path.join(dataset_path, image_file), os.path.join(train_dir, "images", image_file))
		elif image_file in images_test: shutil.copyfile(os.path.join(dataset_path, image_file), os.path.join(test_dir, "images", image_file))
		else: shutil.copyfile(os.path.join(dataset_path, image_file), os.path.join(valid_dir, "images", image_file))

		# Check if aruco is detected. If so, save the label.
		max_area = -10000
		max_idx = -1
		max_corners = []
		max_ids = []

		# Read image
		image = cv2.imread(os.path.join(dataset_path, image_file))

		# Get images width and height
		if first_image:
			IMG_HEIGHT, IMG_WIDTH = image.shape[0], image.shape[1]
			first_image = False

		(corners, ids, rejected) = cv2.aruco.detectMarkers(image, aruco_dict, parameters=aruco_params)

		image_name = image_file.replace('.png', '')

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
					image2detection_data[image_name] = {'image': image, 'corners': corners, 'ids': ids}
					break		

				if areas[-1] > AREA_THRESHOLD and areas[-1] > max_area:
					max_area = areas[-1]
					max_idx = idx
					max_corners = corners
					max_ids = ids
				if len(max_corners) > 0:
					image2detection_data[image_name] = {'image': image, 'corners': max_corners, 'ids': ids[max_idx]}

	for image_name in image2detection_data.keys():
		print(f'[INFO] {image_name} labelled')
		# Get corners, ids and image
		markerCorner = image2detection_data[image_name]["corners"]
		markerID = image2detection_data[image_name]["ids"]
		image = image2detection_data[image_name]["image"]

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
		xmin, ymin, xmax, ymax = topLeft[0] / IMG_WIDTH, topLeft[1] / IMG_HEIGHT, bottomRight[0] / IMG_WIDTH, bottomRight[1] / IMG_HEIGHT
		cv2.imshow("Image", image)
		cv2.waitKey(0)
		# Save label
		image_file = image_name + '.png'
		if image_file in images_train:
			with open(os.path.join(train_dir, "labels", image_name + ".txt"), "w") as label_file: label_file.write(f"0 {xmin} {ymin} {xmax} {ymax}")
		elif image_file in images_test:
			with open(os.path.join(test_dir, "labels", image_name + ".txt"), "w") as label_file: label_file.write(f"0 {xmin} {ymin} {xmax} {ymax}")
		else:
			with open(os.path.join(valid_dir, "labels", image_name + ".txt"), "w") as label_file: label_file.write(f"0 {xmin} {ymin} {xmax} {ymax}")

if __name__ == '__main__':
    try:
        make_dataset()
    except rospy.ROSInterruptException:
        pass