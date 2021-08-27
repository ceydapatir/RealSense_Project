import pyrealsense2.pyrealsense2 as rs
import configparser
import numpy as np
import time
import csv
import cv2

config_pro = configparser.ConfigParser()
config_pro.read('start.ini')
config_pro.sections()

D_cam = config_pro['D435i'].getboolean('Camera')
T_cam = config_pro['T265'].getboolean('Camera')

def Start_pipe():
	global p, cfg

	p = rs.pipeline(ctx) 
	cfg = rs.config()

def Read_dev():
	global dev

	dev_serial = dev.get_info(rs.camera_info.serial_number)
	del dev 
	cfg.enable_device(dev_serial)
	p.start(cfg)

#Start camera D435I
def d_start():
	global dev, T_cam, colorwriter, depthwriter, pipelines_d
	
	Start_pipe()

	D_dweidht = int(config_pro['D435i']['Resolution_DWidht'])
	D_dheight = int(config_pro['D435i']['Resolution_DHeight'])
	D_cweidht = int(config_pro['D435i']['Resolution_CWidht'])
	D_cheight = int(config_pro['D435i']['Resolution_CHeight'])
				
	cfg.enable_stream(rs.stream.accel,rs.format.motion_xyz32f,63)
	cfg.enable_stream(rs.stream.gyro,rs.format.motion_xyz32f,200)
				
	cfg.enable_stream(rs.stream.depth, D_dweidht, D_dheight, rs.format.z16, fps)
	cfg.enable_stream(rs.stream.color, D_cweidht, D_cheight, rs.format.bgr8, fps)	
				
	if video:
				
		color_path = 'color.avi'
		depth_path = 'depth.avi'

		colorwriter = cv2.VideoWriter(color_path, cv2.VideoWriter_fourcc(*'XVID'), fps, (D_cweidht, D_cheight), 1)
		depthwriter = cv2.VideoWriter(depth_path, cv2.VideoWriter_fourcc(*'XVID'), fps, (D_dweidht, D_dheight), 1)
	
	Read_dev()
	pipelines_d = p

#Start camera T265
def t_start():
	global dev, pipelines_t, D_cam, rightwriter, leftwriter
	
	Start_pipe() 

	T_weidht = int(config_pro['T265']['Resolution_Widht'])
	T_height = int(config_pro['T265']['Resolution_Height'])

	cfg.enable_stream(rs.stream.pose)
	cfg.enable_stream(rs.stream.gyro)
	cfg.enable_stream(rs.stream.fisheye, 1)
	cfg.enable_stream(rs.stream.fisheye, 2)

	if video:

		right_path = 'right.avi'
		left_path = 'left.avi'

		rightwriter = cv2.VideoWriter(right_path, cv2.VideoWriter_fourcc(*'XVID'), fps, (T_weidht, T_height), 1)
		leftwriter = cv2.VideoWriter(left_path, cv2.VideoWriter_fourcc(*'XVID'), fps, (T_weidht, T_height), 1)

	Read_dev()
	pipelines_t = p

#Check camera states and start accessible ones
if D_cam or T_cam:
	
	start_cam = True

	fps = int(config_pro['DEFAULT']['FPS'])
	video = config_pro['DEFAULT'].getboolean('Video')
	data_write = config_pro['DEFAULT'].getboolean('Data_Write')
	timer = float(config_pro['DEFAULT']['Time'])
	data_file = config_pro['DEFAULT'].getboolean('Data_File')

	#Append camera states to array
	ar_cam = []
	ctx = rs.context()
	for dev in ctx.query_devices():
		dev.hardware_reset()
		time.sleep(1)
		message = dev.get_info(rs.camera_info.name)
		print(message)
		d_bool = "D435I" in message
		t_bool = "T265" in message
		ar_cam.append(d_bool)
		ar_cam.append(t_bool)

	#Start accessible cameras and block others
	if len(ar_cam)>0:
		if len(ar_cam)>2:
			if (D_cam and T_cam) or not (D_cam and T_cam):
				counter=0
				for dev in ctx.query_devices():
					counter=counter+1
					if counter==1:
						d_start()
					elif counter==2:
						t_start()
			else:
				if D_cam and ar_cam[0]:
					d_start()
					T_cam = False
				elif T_cam and ar_cam[2]:
					t_start()
					D_cam = False
		elif len(ar_cam)<3:
			if D_cam and d_bool:
				d_start()
				T_cam = False
			elif T_cam and t_bool:
				t_start()
				D_cam = False
			else:
				start_cam = False
	else:
		start_cam = False
else:
	start_cam = False

try:
	start_time = time.time()
	due_time = start_time

	while start_cam and (due_time-start_time <= timer):
		
		if T_cam:
						
			frame = pipelines_t.wait_for_frames()
			
			pose = frame.get_pose_frame()
			frameset = frame.as_frameset()
				
			f1 = frameset.get_fisheye_frame(1).as_video_frame()
			f2 = frameset.get_fisheye_frame(2).as_video_frame()
	
			left_data = np.asanyarray(f1.get_data())
			right_data = np.asanyarray(f2.get_data())
			
			if pose:
				data = pose.get_pose_data()
				pos_str = "Position: " + str(data.translation)
				velo_str = "Velocity: " + str(data.velocity)
				taccel_str = "Acceleration: "+ str(data.acceleration)
			
			#If data_write is True in start.ini, write IMU data on frames
			if data_write:		
				cv2.putText(left_data, pos_str, (10,30),cv2.FONT_HERSHEY_PLAIN, 0.8, 200)
				cv2.putText(left_data, velo_str, (10,60),cv2.FONT_HERSHEY_PLAIN, 0.8, 200)
				cv2.putText(left_data, taccel_str, (10,90),cv2.FONT_HERSHEY_PLAIN, 0.8, 200)
					
			cv2.imshow('RealSense_Left', left_data)
			cv2.imshow('RealSense_Right', right_data)

		if D_cam:
			frames = pipelines_d.wait_for_frames()

			a_frame_data = frames[3].as_motion_frame().get_motion_data()
			g_frame_data = frames[2].as_motion_frame().get_motion_data()

			accel = np.asarray([a_frame_data.x, a_frame_data.y, a_frame_data.z], dtype=np.dtype('U25'))
			gyro = np.array([g_frame_data.x, g_frame_data.y, g_frame_data.z], dtype=np.dtype('U25'))
			
			gyro_str = "gyro: x : "+gyro[0]+" y : "+gyro[1]+" z : "+gyro[2]
			accel_str = "accelerometer: x : "+accel[0]+" y : "+accel[1]+" z : "+accel[2]

			depth_frame = frames.get_depth_frame()
			color_frame = frames.get_color_frame()
			
			depth_image = np.asanyarray(depth_frame.get_data())
			color_image = np.asanyarray(color_frame.get_data())

			depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

			#If data_write is True in start.ini, write IMU data on frames
			if data_write:
				cv2.putText(color_image, gyro_str, (10,30),cv2.FONT_HERSHEY_PLAIN, 0.80, 200)
				cv2.putText(color_image, accel_str, (10,60),cv2.FONT_HERSHEY_PLAIN, 0.80, 200)
				
			cv2.imshow('RealSense_Color', color_image)
			cv2.imshow('RealSense_Depth', depth_colormap)
		
		#If video is True in start.ini, save video from active cameras
		if video:
			if D_cam:
				colorwriter.write(color_image)
				depthwriter.write(depth_colormap)
			if T_cam:
				right_data=cv2.cvtColor(right_data,cv2.COLOR_GRAY2BGR) 
				rightwriter.write(right_data)
				left_data=cv2.cvtColor(left_data,cv2.COLOR_GRAY2BGR)
				leftwriter.write(left_data)

		#If data_file is True in start.ini, save data from active cameras to data files
		if data_file:
			if D_cam:
				with open('D435i_Data.csv', mode='a') as D_file:
					D_file_writer = csv.writer(D_file, delimiter=' ', quotechar=' ', quoting=csv.QUOTE_MINIMAL)
					D_file_writer.writerow([gyro_str])
					D_file_writer.writerow([accel_str])
			if T_cam:
				with open('T265_Data.csv', mode='a') as T_file:
					T_file_writer = csv.writer(T_file, delimiter=' ', quotechar=' ', quoting=csv.QUOTE_MINIMAL)
					T_file_writer.writerow([pos_str])
					T_file_writer.writerow([velo_str])				
					T_file_writer.writerow([taccel_str])
		
		due_time=time.time()
		cv2.waitKey(1)

finally:
	if start_cam:
		if D_cam:
			pipelines_d.stop()

		if T_cam:
			pipelines_t.stop()


