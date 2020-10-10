import pyzed.sl as sl
import numpy as np
import cv2
import time
#((195,231,95))
#((131,119,33))

class ObjectTracker(object):
	def __init__(self, testing = False, video_filename = None):
		#self.LOWER = np.array((147, 156, 79))
		#self.UPPER = np.array((205,209,100))
		self.LOWER = np.array((131, 119, 33))
		self.UPPER = np.array((170,190,80))
		self.MIN_RADIUS = 10
		self.FRAME_RATE = 10
		self.ball_in_frame = None
		self.center = None
		self.radius = 0
		self.firstRun = 0
		self.testing = testing
		self.leftCV = None
		self.rightCV = None
		self.depth = None
		self.depthCV = None
		self.pc = None
		self.pcCV = None
		self.runtime_parameters = sl.RuntimeParameters()
		finalVideoOut = "../logs/objtracker" + time.strftime("%Y%m%d-%H%M%S") # log the video with tennis ball circles
		try:
			self.camera = sl.Camera()
			print("camera set")
			self.init_params = sl.InitParameters()
			print("found params")
			self.init_params.camera_resolution = sl.RESOLUTION.RESOLUTION_HD1080
			print("resolution set")
			print("fps set")
			err = self.camera.open(self.init_params)
			print("camera opened")
			if err != sl.ERROR_CODE.SUCCESS:
				print("\n\n\nFAILED TO OPEN CAMERA\n\n\n")
		except Exception:
			raise Exception("FAILED TO OPEN CAMERA")
		pass
		image_size = self.camera.get_resolution()
		self.left = sl.Mat(image_size.width/2, image_size.height, sl.MAT_TYPE.MAT_TYPE_8U_C4)
		self.right = sl.Mat(image_size.width/2, image_size.height, sl.MAT_TYPE.MAT_TYPE_8U_C4) # these need to be declared here so we properly retrieve information from the camera
		#self.depth = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.MAT_TYPE_8U_C4)
		#self.pc = sl.Mat()
		if self.camera.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
			self.camera.retrieve_image(self.left, sl.VIEW.VIEW_LEFT, sl.MEM.MEM_CPU, int(1920), int(1080))
			self.camera.retrieve_image(self.right, sl.VIEW.VIEW_RIGHT, sl.MEM.MEM_CPU, int(1920), int(1080))
		else:
			print("Frame Capture Failed")
		
		fourcc = cv2.VideoWriter_fourcc(*'XVID')
		if video_filename is None:
			video_filename = "logs/objtracker" + time.strftime("%Y%m%d-%H%M%S") # save videos to unique files.
		print(video_filename)
		self.video_out_left = cv2.VideoWriter(video_filename + "_left.avi", fourcc, self.FRAME_RATE, (1920, 1080))
		assert(self.video_out_left.isOpened())
		#self.video_out_right = cv2.VideoWriter(video_filename + "_right.avi", fourcc, self.FRAME_RATE, (1920,1080))
		#assert(self.video_out_right.isOpened())
		
		#self.finalVideoOutLeft = cv2.VideoWriter(finalVideoOut + "_finalleft.avi", fourcc, self.FRAME_RATE, (1920,1080))
		#assert(self.finalVideoOutLeft.isOpened())
		#self.finalVideoOutRight = cv2.VideoWriter(finalVideoOut + "_finalright.avi", fourcc, self.FRAME_RATE, (1920,1080))
		#assert(self.finalVideoOutRight.isOpened())


	def __del__(self):
		self.video_out_left.release()
		self.video_out_right.release()
		#self.finalVideoOutLeft.release()
		#self.finalVideoOutRight.release()
		self.camera.close() # need to use close because this is a ZED api object not an OpenCV object
	
	def track_ball(self):
		#time.sleep(0.033)
		self.ball_in_frame = False
		if self.camera.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
			print("grabbed frame")			
			self.camera.retrieve_image(self.left, sl.VIEW.VIEW_LEFT, sl.MEM.MEM_CPU, int(1920), int(1080))
			#self.camera.retrieve_image(self.right, sl.VIEW.VIEW_RIGHT, sl.MEM.MEM_CPU, int(1920), int(1080))
			self.leftCV = self.left.get_data()
			#self.rightCV = self.right.get_data()
			#self.camera.retrieve_image(self.depth, sl.VIEW.VIEW_DEPTH)
			#self.depthCV = self.depth.get_data()
			#self.camera.retrieve_measure(self.pc, sl.MEASURE.MEASURE_XYZRGBA, sl.MEM.MEM_CPU, int(1920), int(1080))
			#self.pcCV = self.pc.get_data()
			
			cv2.waitKey(1)
			#print(self.leftCV)
			#cv2.imshow("left", self.leftCV)
			self.leftCV = cv2.cvtColor(self.leftCV, cv2.COLOR_RGBA2RGB)
			#self.rightCV = cv2.cvtColor(self.rightCV, cv2.COLOR_RGBA2RGB)
			#print(self.leftCV)
			#self.video_out_left.write(self.leftCV)
			#cv2.imshow("right", self.rightCV)
			#self.video_out_right.write(self.rightCV)
			#hsv = cv2.cvtColor(self.leftCV, cv2.COLOR_RGB2BGR)
			#hsv = cv2.cvtColor(hsv, cv2.COLOR_RGB2HSV)
			#cv2.imshow("hsv", hsv)
			#mask = cv2.inRange(hsv, self.LOWER, self.UPPER)
			#cv2.imshow("mask", mask)
			#cv2.imshow("depth", self.depthCV)
			#cv2.imshow("point cloud", self.pcCV)
			self.video_out_left.write(self.leftCV)
			#self.video_out_right.write(self.rightCV)
			if not self.firstRun:
				self.firstRun = 1
				#cv2.imwrite("test.png", hsv)
			return self.ball_in_frame, (0,0), 0
		else:
			return self.ball_in_frame, (0,0), 0	



if __name__ == '__main__':
	tracker = ObjectTracker(True, "../logs/objtracker" + time.strftime("%Y%m%d-%H%M%S"))
	while True:
		ball_in_frame, center, radius = tracker.track_ball()
		if ball_in_frame:
			if radius == 0:
				radius = 1
			print("ball found at %s, distance %s" % (center, 1.0/radius))
		else:
			print("No ball found")
