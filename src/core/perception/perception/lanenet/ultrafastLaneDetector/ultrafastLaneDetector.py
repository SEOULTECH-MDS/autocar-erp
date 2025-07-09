import cv2
import torch
import scipy.special
import numpy as np
import torchvision
import torchvision.transforms as transforms
from PIL import Image
from enum import Enum
from scipy.spatial.distance import cdist
from sklearn.linear_model import RANSACRegressor
from scipy.interpolate import CubicSpline, interp1d

from perception.lanenet.ultrafastLaneDetector.model import parsingNet
from perception.lanenet.ultrafastLaneDetector.perspective_transformation import *

lane_colors = [(0,0,255),(0,255,0),(255,0,0),(0,255,255)]# 파란색, 녹색, 빨간색, 청록색

tusimple_row_anchor = [ 64,  68,  72,  76,  80,  84,  88,  92,  96, 100, 104, 108, 112,
			116, 120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160, 164,
			168, 172, 176, 180, 184, 188, 192, 196, 200, 204, 208, 212, 216,
			220, 224, 228, 232, 236, 240, 244, 248, 252, 256, 260, 264, 268,
			272, 276, 280, 284]
culane_row_anchor = [121, 131, 141, 150, 160, 170, 180, 189, 199, 209, 219, 228, 238, 248, 258, 267, 277, 287]

filtered_angle = 0.0
past_mode = 'pass'
slope1 = 0.0
slope2 = 0.0
intercept1 = 0.0
intercept2 = 0.0
count_lane1 = []
count_lane2 = []
slope_array1 =[]
slope_array2 =[]
lane_width = 500

def PT_draw_line(transform, slope1, intercept1, slope2, intercept2):
	#이미지 생성 cv2 형식
	height = 590
	width = 1640
	color = 255
	blank_binary_image = np.zeros((height, width), np.uint8)
	#선 그리기
	cv2.line(blank_binary_image, (int((590-intercept1)/slope1),590) , (int((295-intercept1)/slope1),295)  , color, 2)
	cv2.line(blank_binary_image, (int((590-intercept2)/slope2),590) , (int((295-intercept2)/slope2),295)  , color, 2)
	#np 형식으로 바꾸기
	PT_image = transform.forward(blank_binary_image)

	return PT_image
	#(647, 300), (302,589), (1357,589), (993,300)  >> (620,0), (620,590), (1120,590), (1120,0)

def chage_one_lane_mode(lanes_detected, slope1, slope2):  #change moed with lane recognition percent array changing between lane1 mode and lane2 mode, 기울기 변화

	global count_lane1 ,count_lane2, lane_rcg_pct, slope_array1, slope_array2, past_mode
	slope1_diff = None
	slope2_diff = None

	#change mode with lane recognition percent

	if lanes_detected[1] and lanes_detected[2]:
		mode = 'both'
	elif lanes_detected[1]:
		slope_array1.append(slope1)
		mode = 'lane1'
	elif lanes_detected[2]:
		slope_array2.append(slope2)
		mode = 'lane2'
	else:
		mode = 'pass'
	# 알고리즘은 나중에 짜자

	# #get slope diff from past_mode
	# if len(slope_array1) > 5 and len(slope_array2) >5:
	# 	if mode == 'both':
	# 		if past_mode == 'both':
	# 			slope1_diff = abs(slope_array1[-2] - slope_array1[-1])
	# 			slope2_diff = abs(slope_array2[-2] - slope_array2[-1])
	# 			if slope1_diff < 0.1 and slope2_diff < 0.1:
	# 				mode = 'both'
	# 			elif slope1_diff >= 0.1:
	# 				mode = 'lane2'
	# 			elif slope2_diff >= 0.1:
	# 				mode = 'lane1'
	# 			else:
	# 				mode = 'past'
	# 		elif past_mode == 'lane1':
	# 			slope1_diff = abs(slope_array1[-2] - slope_array1[-1])
	# 			if slope1_diff < 0.1:
	# 				mode = 'both'
	# 			else:
	# 				mode = ''


	# else:
	# 	mode = 'past'
	# # if lanes_detected[1]:
	# # 	count_lane1.append(1)
	# # else:
	# # 	count_lane1.append(0)

	# # if lanes_detected[2]:
	# # 	count_lane2.append(1)
	# # else:
	# # 	count_lane2.append(0)

	# # if len(count_lane1) > 10: #lane mode 판별할 주기. 16Hz 참고
	# # 	del count_lane1[0]
	# # if len(count_lane2) > 10:
	# # 	del count_lane2[0]

	# # ct_lane_sum1 = sum(count_lane1)
	# # ct_lane_sum2 = sum(count_lane2)

	# # if ct_lane_sum1 >= ct_lane_sum2:
	# # 	lane_rcg_pct = ct_lane_sum2/ct_lane_sum1
	# # 	if lane_rcg_pct < 0.5:
	# # 		mode = 'lane1'
	# # 	else:
	# # 		mode = 'both'
	# # else:
	# # 	lane_rcg_pct = ct_lane_sum1/ct_lane_sum2
	# # 	if lane_rcg_pct < 0.5:
	# # 		mode = 'lane2'
	# # 	else:
	# # 		mode = 'both'

	# #change mode with lane slope change #slope 차이 0.1 보다 낮아질 때 까지 아닌 lane 쓰기

	# # slope_array1.append(slope1)
	# # slope_array2.append(slope2)


	# if len(slope_array1) > 2: #lane mode 판별할 주기. 16Hz 참고
	# 	del slope_array1[0]
	# 	slope1_diff =  abs(slope_array1[1] - slope_array1[0])
	# if len(slope_array2) > 2:
	# 	del slope_array2[0]
	# 	slope2_diff =  abs(slope_array2[1] - slope_array2[0])

	# if mode != 'stop':
	# 	if slope1_diff != None and slope2_diff != None:
	# 		if slope1_diff > 0.1 and slope2_diff > 0.1:
	# 			mode = 'stop'
	# 		elif slope1_diff > 0.1:
	# 			mode = 'lane2'
	# 		elif slope2_diff > 0.1:
	# 			mode = 'lane1'
	# 		else:
	# 			mode = 'both'

	# print(mode)

	past_mode = mode

	return mode

		# print("lane_rcg_pct",lane_rcg_pct)

	# if len(slope_array1) > 1:
	# 	print("slope1_dff", slope_array1[1] - slope_array1[0])
	# if len(slope_array2) > 1:
	# 	print("slope2_dff", slope_array2[1] - slope_array2[0])

	# if lane_rcg_pct < 0.5:
	# 	return True
	# else:
	# 	return False

def get_detected_points(mode,lanes_points):
	if mode == 'both':
		points = lanes_points[1] + lanes_points[2]
	elif mode == 'lane1':
		points = lanes_points[1]
	elif mode == 'lane2':
		points = lanes_points[2]
	else:
		points = []
	return points


def perspective_transform(transform, lane_index, lane_points):
	#create image of points
	points = list(map(np.array, lane_points[lane_index]))
	height = 590
	width = 1640
	point_img = np.zeros((height, width), np.uint8)
	for point in points:
		x, y = point
		# point_img[y,x] = 255
		point_img[y,x] = 255
	#perspective transform
	tf_points_img = transform.forward(point_img)
	return tf_points_img

def tf_img_to_points(tf_points_img):


	converted_points = np.where(tf_points_img == 255)
	#print('converted_points',converted_points)
	y_coords_list = converted_points[0].tolist()
	x_coords_list = converted_points[1].tolist()

	# [x, y] 형태의 2차원 리스트 생성
	converted_points = [[x, y] for x, y in zip(x_coords_list, y_coords_list)]
	# converted_points = list(zip(*converted_points))
	# converted_points = list(zip(converted_points[1], converted_points[0]))
	#밝기 확인
	# for x, y in coordinates:
	# 	print(tf_points_img[y,x])
	# print(indices)


	# converted_points = np.array(converted_points)
	# print('2',converted_points)
	return converted_points

def sort_points(converted_points):


	height = 590
	width = 1640
	ax = []
	ay = []

	# 같은 y값 x값으로 평균 때리기
	y_values = {}
	for x, y in converted_points:
		if y in y_values:
			y_values[y].append(x)
		else:
			y_values[y] = [x]

	# 각 y 값에 대한 x 값의 평균을 계산하여 새로운 리스트 생성
	converted_points = []
	for y, x_list in y_values.items():
		average_x = sum(x_list) / len(x_list)
		converted_points.append((average_x, y))

	#sort y올림차순
	sorted_points = sorted(converted_points, key=lambda x: x[1])
	return sorted_points


def get_path_and_Lateral_error(sorted_points):
	height = 590
	width = 1640
	path = []
	result = process_lane_for_uichan_work_for_uichan(sorted_points) # get slope and intercept
	if result != 'pass':
		slope, intercept = result
		# get lateral error
		lateral_pose  = int((height-intercept)/slope)
		lateral_error = -(1197 - lateral_pose)
		# get path
		yy = np.arange(0, height, 1)
		for y in yy:
			path.append((int((y-intercept)/slope),y))
		mid_line_img = np.zeros((height, width), np.uint8)
		for x, y in path:
			if 0 <= x < width and 0 <= y < height:
				mid_line_img[int(y), int(x)] = 255
			else: break
		path = transfrom_coordinates(path)


		return lateral_error, path, mid_line_img

	else:
		return 'pass'

def transfrom_coordinates(path):
		# #transform_coordinate / 차량 중심 x,y 좌표료 변환
	center_x = 939 #midpoint of car, gonna be Base of coordinate system
	center_y = 590 #gonna be Base of coordinate system
	converted_path = []
	for x, y in path:
		converted_path.append((x-center_x,center_y-y))


	return converted_path

def cubic_spline(converted_points):
	if converted_points != []:
		height = 590
		width = 1640
		ax = []
		ay = []

		# 같은 y값 x값으로 평균 때리기
		y_values = {}
		for x, y in converted_points:
			if y in y_values:
				y_values[y].append(x)
			else:
				y_values[y] = [x]

		# 각 y 값에 대한 x 값의 평균을 계산하여 새로운 리스트 생성
		converted_points = []
		for y, x_list in y_values.items():
			average_x = sum(x_list) / len(x_list)
			converted_points.append((average_x, y))



		#sort y올림차순
		sorted_points = sorted(converted_points, key=lambda x: x[1])
		for x,y in sorted_points:
			ax.append(x)
			ay.append(y)

		#print('converted',converted_points)
		#print(ay)
		cs = CubicSpline(ay,ax)
		# quadratic_interp = interp1d(ay, ax, kind='linear')
		cy = np.arange(0, height, 1)
		# cy = np.arange(min(ay), max(ay) + 1, 1)
		cx = cs(cy)

		#make image after cubicspline
		cubic_img = np.zeros((height, width), np.uint8)
		for x, y in zip(cx, cy):
			if 0 <= x < width and 0 <= y < height:
				cubic_img[int(y), int(x)] = 255

		return cx, cy, cubic_img
	return 'fail'

def process_lane_for_uichan_work_for_uichan(sorted_points):

	arctan = calculate_arctan(sorted_points)
	dev = np.round(arctan - np.mean(arctan))
	var = np.mean(dev**2)
	#print("L{}_dev".format(lane_index), dev, "L{}_var".format(lane_index), var)
	slope = None
	intercept = None
	#print('var',var)
	# if var > 100 :
	# 	pass

	# elif var > 50 :
	# 	sorted_points = remove_outlier(sorted_points, dev)
	# 	slope, intercept = draw_polyfit_line([], sorted_points, [])
	# else :

	slope, intercept = draw_polyfit_line([], sorted_points, [])

	if slope !=None and intercept != None:
		return  slope, intercept
	else:
		return 'pass'
	# return cross_x, slope, intercept


def process_lane(lanes_points):
	# arctan = calculate_arctan(lanes_points)
	# dev = np.round(arctan - np.mean(arctan))
	# var = np.mean(dev**2)
	#print("L{}_dev".format(lane_index), dev, "L{}_var".format(lane_index), var)
	slope = None
	intercept = None
	# print('var',var)
	# if var > 100 :
	# 	return 'fail'
	# elif var > 50 :
	# 	lanes_points = remove_outlier(lanes_points, dev)
	# 	slope, intercept = Ransac(lanes_points)
	try:
		slope, intercept = Ransac(lanes_points)
	except:
		pass

	if slope !=None and intercept != None:
		return slope, intercept
	else:
		return 'fail'
	# return cross_x, slope, intercept

def remove_outlier(lanes_points, data):
	data_mean = np.mean(np.abs(data))
	#print('data_mean',data_mean)
	outlier_index = np.where(np.abs(data)< data_mean)
	# print(outlier_index)
	cleaned_lanes_points = np.delete(lanes_points, outlier_index, axis=0)
	return cleaned_lanes_points

def Ransac(points):

	points_x, points_y = np.transpose(points)
	# points_x = points[:, 0]
	# points_y = points[:, 1]

	# RANSAC 모델 생성
	ransac = RANSACRegressor()

	# X와 y를 2D 배열로 변환
	x = points_x.reshape(-1, 1)
	y = points_y.reshape(-1, 1)

	# RANSAC을 사용하여 선형 회귀 모델 학습
	try:
		ransac.fit(x, y)
	except:
		slope = None
		intercept = None
		return slope, intercept

	# # RANSAC 모델로부터 추정된 선형 회귀식의 기울기와 y 절편 가져오기
	slope = ransac.estimator_.coef_[0][0]
	intercept = ransac.estimator_.intercept_[0]

	if slope > 2000000000:
		slope = 2000000000
	elif slope < -2000000000:
		slope = -2000000000

	return slope, intercept

#def draw_polyfit_line(visualization_img, points, color):
#	points_x = points[:, 0]
#	points_y = points[:, 1]
#	coefficients = np.polyfit(points_x, points_y, 1)
#	f1 = np.poly1d(coefficients)
#
#	start_x = points[0, 0]
#	end_x = points[-1, 0]
#	start_y = f1(start_x)
#	end_y = f1(end_x)
#	start_point = (int(start_x), int(start_y))
#	end_point = (int(end_x), int(end_y))
#	cv2.line(visualization_img, start_point, end_point, color, 2)

def find_point_over_dcsion_line(slope, intercept, dcsion_y):
	dcsion_x = (dcsion_y - intercept) / slope
	return dcsion_x

def calculate_arctan(points):
	#print('points', points)
	x, y = np.transpose(points)
	dx = np.diff(x)
	dy = np.diff(y)
	#print('x:', x)
	#print('dx:', dx)
	#print('dy:', dy)
	arctan_values = np.round(np.degrees(np.arctan2(dy, dx)), 1)
	return arctan_values

def pixel_to_meter(pixel):

    if isinstance(pixel, list):  # pixel이 리스트인 경우
        meter = [(x * 0.013636, y * 0.013636) for x, y in pixel]
    else:  # pixel이 리스트가 아닌 경우 (예: 단일 숫자)
        meter = pixel * 0.013636
    return meter

class ModelType(Enum):
	TUSIMPLE = 0
	CULANE = 1

class ModelConfig():

	def __init__(self, model_type):

		if model_type == ModelType.TUSIMPLE :
			self.init_tusimple_config()
		else:
			self.init_culane_config()

		self.filtered_angle = 0.0

	def init_tusimple_config(self):
		self.img_w = 1280
		self.img_h = 720
		self.row_anchor = tusimple_row_anchor
		self.griding_num = 100
		self.cls_num_per_lane = 56

	def init_culane_config(self):
		self.img_w = 1640
		self.img_h = 590
		self.row_anchor = culane_row_anchor
		self.griding_num = 200
		self.cls_num_per_lane = 18

class UltrafastLaneDetector():

	def __init__(self, model_path, model_type=ModelType.TUSIMPLE, use_gpu=False):

		self.use_gpu = use_gpu

		# Load model configuration based on the model type
		self.cfg = ModelConfig(model_type)

		# Initialize model
		self.model = self.initialize_model(model_path, self.cfg, use_gpu)

		# Initialize image transformation
		self.img_transform = self.initialize_image_transform()



	@staticmethod
	def initialize_model(model_path, cfg, use_gpu):

		# Load the model architecture
		net = parsingNet(pretrained = False, backbone='18', cls_dim = (cfg.griding_num+1,cfg.cls_num_per_lane,4),
						use_aux=False) # we dont need auxiliary segmentation in testing


		# Load the weights from the downloaded model
		if use_gpu:
			net = net.cuda()
			state_dict = torch.load(model_path, map_location='cuda')['model'] # CUDA
		else:
			state_dict = torch.load(model_path, map_location='cpu')['model'] # CPU

		compatible_state_dict = {}
		for k, v in state_dict.items():
			if 'module.' in k:
				compatible_state_dict[k[7:]] = v
			else:
				compatible_state_dict[k] = v

		# Load the weights into the model
		net.load_state_dict(compatible_state_dict, strict=False)
		net.eval()

		return net

	@staticmethod
	def initialize_image_transform():
		# Create transfom operation to resize and normalize the input images
		img_transforms = transforms.Compose([
			transforms.Resize((288, 800)),
			transforms.ToTensor(),
			transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
		])

		return img_transforms

	def detect_lanes(self, image, draw_points=True):

		input_tensor = self.prepare_input(image)

		# Perform inference on the image
		output = self.inference(input_tensor)

		# Process output data
		self.lanes_points, self.lanes_detected = self.process_output(output, self.cfg)

		# Draw depth image
		final_detected, line_img, lateral_error, list_for_rviz = self.draw_lanes(self.lanes_points, self.lanes_detected)

		return final_detected, line_img, lateral_error, list_for_rviz

	def prepare_input(self, img):
		# Transform the image for inference
		img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		img_pil = Image.fromarray(img)
		input_img = self.img_transform(img_pil)
		input_tensor = input_img[None, ...]

		if self.use_gpu:
			input_tensor = input_tensor.cuda()

		return input_tensor

	def inference(self, input_tensor):
		with torch.no_grad():
			output = self.model(input_tensor)

		return output



	@staticmethod
	def process_output(output, cfg):
		# Parse the output of the model
		processed_output = output[0].data.cpu().numpy()
		processed_output = processed_output[:, ::-1, :]
		prob = scipy.special.softmax(processed_output[:-1, :, :], axis=0)
		idx = np.arange(cfg.griding_num) + 1
		idx = idx.reshape(-1, 1, 1)
		loc = np.sum(prob * idx, axis=0)
		processed_output = np.argmax(processed_output, axis=0)
		loc[processed_output == cfg.griding_num] = 0
		processed_output = loc


		col_sample = np.linspace(0, 800 - 1, cfg.griding_num)
		col_sample_w = col_sample[1] - col_sample[0]

		lanes_points = []
		lanes_detected = []

		max_lanes = processed_output.shape[1]
		for lane_num in range(max_lanes):
			lane_points = []
			# Check if there are any points detected in the lane
			if np.sum(processed_output[:, lane_num] != 0) > 2:

				lanes_detected.append(True)

				# Process each of the points for each lane
				for point_num in range(processed_output.shape[0]):
					if processed_output[point_num, lane_num] > 0:
						lane_point = [int(processed_output[point_num, lane_num] * col_sample_w * cfg.img_w / 800) - 1, int(cfg.img_h * (cfg.row_anchor[cfg.cls_num_per_lane-1-point_num]/288)) - 1 ]
						lane_points.append(lane_point)
			else:
				lanes_detected.append(False)

			lanes_points.append(lane_points)
		return np.array(lanes_points, dtype=object), np.array(lanes_detected, dtype=object)

	@staticmethod
	def draw_lanes(lanes_points, lanes_detected):
		global slope_array1, slope_array2, slope1, intercept1, slope2, intercept2, lane_width

		transform = PerspectiveTransformation()
		lane1_detected = False
		lane2_detected = False
		lane1_allowed = False
		lane2_allowed = False
		list_for_rviz = []
		final_detected = False
		height = 590
		width = 1640
		slope1_diff = 1
		slope2_diff = 1
		lateral_error = 10000
		line_img = np.zeros((height, width), np.uint8)
		# print(type(lanes_points))
		# lanes_points = list(map(np.array, lanes_points))
		# print(type(lanes_points))
		#check lane detected and get slope and intercept
		if lanes_detected[1]:
			BVpoint_image = perspective_transform(transform, 1, lanes_points)
			BVpoints = tf_img_to_points(BVpoint_image)
			#print('BVpoints',BVpoints)
			result = process_lane(BVpoints)
			#print('result', result)
			if result != 'fail':
				slope1, intercept1 = result
				slope_array1.append(slope1)
				lane1_detected = True
			else:
				lane1_detected = False

		if lanes_detected[2]:
			BVpoint_image = perspective_transform(transform, 2, lanes_points)
			BVpoints = tf_img_to_points(BVpoint_image)
			result = process_lane(BVpoints)
			if result != 'fail':
				slope2, intercept2 = result
				slope_array2.append(slope2)
				lane2_detected = True
			else:
				lane2_detected = False
		#print('lane1',lanes_detected[1])
		#print('lane2',lanes_detected[2])
		#print('lane1_detected', lane1_detected)
		#print('lane2_detected', lane2_detected)

		#check lane is usable
		if len(slope_array1) > 5: #lane mode 판별할 주기. 16Hz 참고
			del slope_array1[0]
			slope1_diff =  abs(slope_array1[-2] - slope_array1[-1])
		if len(slope_array2) > 5:
			del slope_array2[0]
			slope2_diff =  abs(slope_array2[-2] - slope_array2[-1])

		# print('slope1_diff',slope1_diff)
		# print('slope2_diff',slope2_diff)

		if slope1_diff < 10 and lane1_detected:
			lane1_allowed = True
		else:
			lane1_allowed = False

		if slope2_diff < 10 and lane2_detected:
			lane2_allowed = True
		else:
			lane2_allowed = False

		#print('lane1_allowed', lane1_allowed)
		#print('lane2_allowed', lane2_allowed)
		#get viz image, lateral error, points for rviz
		if lane1_allowed and lane2_allowed:
			#lateral error
			try:
				lane1_height_x  = int((height-intercept1)/slope1)
				lane2_height_x = int((height-intercept2)/slope2)
				lane1_0_x = int((0-intercept1)/slope1)
				lane2_0_x = int((0-intercept2)/slope2)
				lane_midpoint = (lane2_height_x + lane1_height_x)/2
				lane_width = lane2_height_x - lane1_height_x
				#print('lane_width', lane_width)
				lateral_error = -(939 - lane_midpoint)
				#print('lane_midpoint',lane_midpoint)
				#get_viz_image
				line_img = np.zeros((height, width), np.uint8)
				cv2.line(line_img, (lane1_height_x,height) , (lane1_0_x,0)  , 255, 2)
				cv2.line(line_img, (lane2_height_x,height) , (lane2_0_x,0)  , 255, 2)
				#list for rviz line # x1,y1,x2,y2,x3,y3,x4,y4
				list_for_rviz = [(lane1_height_x,height),(lane1_0_x,0),(lane2_0_x,0),(lane2_height_x,height)]
				list_for_rviz = transfrom_coordinates(list_for_rviz)

				final_detected = True
			except:
				pass

		elif lane1_allowed:
			#lateral error
			try:
				lane1_height_x  = int((height-intercept1)/slope1)
				lane2_height_x = lane1_height_x + lane_width
				lane1_0_x = int((0-intercept1)/slope1)
				lane2_0_x = lane1_0_x + lane_width
				lane_midpoint = lane1_height_x + lane_width/2
				lateral_error = -(939 - lane_midpoint)
				#get_viz_image
				line_img = np.zeros((height, width), np.uint8)
				cv2.line(line_img, (lane1_height_x,height) , (lane1_0_x,0)  , 255, 2)
				cv2.line(line_img, (lane2_height_x,height) , (lane2_0_x,0)  , 255, 2)
				#list for rviz line #  x1,y1,x2,y2,x4,y4,x3,y3
				list_for_rviz = [(lane1_height_x,height),(lane1_0_x,0),(lane2_0_x,0),(lane2_height_x,height)]
				list_for_rviz = transfrom_coordinates(list_for_rviz)

				final_detected = True
			except:
				pass

		elif lane2_allowed:
			try:

				#lateral error
				lane2_height_x = int((height-intercept2)/slope2)
				lane1_height_x  = lane2_height_x - lane_width
				lane2_0_x = int((0-intercept2)/slope2)
				lane1_0_x = lane2_0_x - lane_width
				lane_midpoint = lane2_height_x - lane_width/2
				lateral_error = -(939 - lane_midpoint)
				#get_viz_image
				line_img = np.zeros((height, width), np.uint8)
				cv2.line(line_img, (lane1_height_x,height) , (lane1_0_x,0)  , 255, 2)
				cv2.line(line_img, (lane2_height_x,height) , (lane2_0_x,0)  , 255, 2)
				#list for rviz line # x1,y1,x2,y2,x4,y4,x3,y3
				list_for_rviz = [(lane1_height_x,height),(lane1_0_x,0),(lane2_0_x,0),(lane2_height_x,height)]
				list_for_rviz = transfrom_coordinates(list_for_rviz)

				final_detected = True
			except:
				pass

		else:
			final_detected = False
			line_img = np.zeros((height, width), np.uint8)


		list_for_rviz = pixel_to_meter(list_for_rviz)
		lateral_error = pixel_to_meter(lateral_error)
		print('lateral_error',lateral_error)
		#print('list_for_rviz', list_for_rviz)

		return final_detected, line_img, lateral_error, list_for_rviz