import pyrealsense2 as rs
import time

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
#标定时候需要修改fps为30
cfg = pipeline.start(config)
time.sleep(1)
profile = cfg.get_stream(rs.stream.depth)
# profile = cfg.get_stream(rs.stream.color)
intr = profile.as_video_stream_profile().get_intrinsics()
print(intr)  
#  depth内参 --> width:640, height:480, ppx:320.309, ppy:237.548, fx:384.416, fy:384.416, model:Brown Conrady, coeffs:[0 0 0 0 0]
#  color内参 --> width:640, height:480, ppx:326.419, ppy:238.673, fx:610.51, fy:610.54, model:Inverse Brown Conrady, coeffs:[0 0 0 0 0]

'''
内参矩阵
K = fx		s		x0
   	  0		fy		y0
	  0		0		1
'''
#fx,fy为焦距,一般情况下二者相等
 #[fx 0 ppx ; 0 fy ppy ; 0 0 1] 