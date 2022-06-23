#! /usr/bin/env python
# encoding: utf-8
import rospy
from visualization_msgs.msg import Marker
import pyrealsense2 as rs
import multiprocessing as mp
import cv2
import numpy as np
from box import Box
import open3d as o3d
#from meter import Meter
from seg import Tester
import time
from read_pcd import process_round2                      


def distance_to_camera(perWidth, knownWidth = 24, focalLength = 610.5):
    pass


def  aabb2marker(bbox_info):
    marker = Marker(type=Marker.CUBE,ns='zhangaiwu', action=Marker.ADD)
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()
    marker.pose.position.x = bbox_info.oritation.x
    marker.pose.position.y = bbox_info.oritation.y
    marker.pose.position.z = bbox_info.oritation.z
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = bbox_info.position.x              
    marker.scale.y = bbox_info.position.y
    marker.scale.z = bbox_info.position.z
    marker.color.a = 1.0
    marker.color.r = 0.8
    marker.color.g = 0.2 
    marker.color.b = 0.9    
    marker.lifetime = rospy.Duration.from_sec(0.1)
    return marker

   


def draw_border(img, boxes, color =(255, 0, 255)):
    x1, y1, x4 ,y4  = boxes #左上，右下
    x2, y2, x3, y3 = x1, y4, x4, y1  #左下， 右上
    box_h, box_w = y4 - y1, x4 - x1
    # center_x,  center_y = int((x4+x1)/2), int((y4+y1)/2) #  center
    center_x,  center_y = int(x1 + box_w/2), int(y1 + box_h/2)
    cv2.circle(img, (center_x, center_y), 3, color, -1)    
    line_length = int(min(box_h, box_w) / 7)
    # print(line_length)
    # cv2.circle(img, (x1, y1), 3, (255, 0, 255), -1)    #  top_left
    # cv2.circle(img, (x2, y2), 3, (255, 0, 255), -1)    #  bottom-left
    # cv2.circle(img, (x3, y3), 3, (255, 0, 255), -1)    #  top-right
    # cv2.circle(img, (x4, y4), 3, (255, 0, 255), -1)    #  bottom-right

    cv2.line(img, (x1, y1), (x1 , y1 + line_length), color, 2)  #  top-left
    cv2.line(img, (x1, y1), (x1 + line_length , y1), color, 2)

    cv2.line(img, (x2, y2), (x2 , y2 - line_length), color, 2)  #  bottom-left
    cv2.line(img, (x2, y2), (x2 + line_length , y2), color, 2)

    cv2.line(img, (x3, y3), (x3 - line_length, y3),color, 2)  #  top-right
    cv2.line(img, (x3, y3), (x3, y3 + line_length),color, 2)

    cv2.line(img, (x4, y4), (x4 , y4 - line_length), color, 2)  #  bottom-right
    cv2.line(img, (x4, y4), (x4 - line_length , y4), color, 2)                


class Producer(mp.Process):

    def __init__(self, q, preset:int):
        super().__init__()
        self.q = q
        self.preset = preset
        '''
        深度模式设置 区间[0,5],The default is 3.
            常用  3:高精度模式
                  4:高密度模式
                  5:中密度模式
        '''

    def run(self):
        colorizer = rs.colorizer() #深度成像
        #pc = rs.pointcloud()
        threshold = rs.threshold_filter()
        threshold.set_option(rs.option.min_distance,0.3)
        threshold.set_option(rs.option.max_distance,2.0)
        spatial = rs.spatial_filter() #空间滤波
        #spatial.set_option(rs.option.filter_magnitude,5)
        spatial.set_option(rs.option.holes_fill,0)
        spatial.set_option(rs.option.filter_smooth_alpha,0.5)
        spatial.set_option(rs.option.filter_smooth_delta,20)
        # spatial.set_option(rs.option.holes_fill,2)
        # decimation = rs.decimation_filter()
        # decimation.set_option(rs.option.filter_magnitude,2)
        temporal = rs.temporal_filter() #时间滤波
        temporal.set_option(rs.option.filter_smooth_alpha,0.4)
        temporal.set_option(rs.option.filter_smooth_delta,20)
        hole_filling = rs.hole_filling_filter() #孔填充核
        hole_filling.set_option(rs.option.holes_fill,1)
  


        pipeline = rs.pipeline()
        config = rs.config() #配置文件
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) #1280,720
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
        # config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)

        # Start streaming
        profile = pipeline.start(config)
        # 设置彩色相机
        device = profile.get_device()
        color_sensor= device.query_sensors()[1] 
        color_sensor.set_option(rs.option.sharpness,100) # 设置锐度为100(最大)
        
        # 设置深度模式
        depth_sensor=device.first_depth_sensor()
        depth_sensor.set_option(rs.option.visual_preset,self.preset)
     
        depth_scale = device.first_depth_sensor().get_depth_scale() # 获取深度传感器的深度标尺
        align_to = rs.stream.color
        align = rs.align(align_to)
        self.q.put(depth_scale)
        while True:
            try:
                frames = pipeline.wait_for_frames()
                frames = threshold.process(frames)
                frames = spatial.process(frames)
                # # # depth_frame = decimation.process(depth_frame)
                frames = temporal.process(frames)
                frames = hole_filling.process(frames)

                frames=frames.as_frameset()
                aligned_frames = align.process(frames)

                depth_frame = aligned_frames.get_depth_frame() #获取深度图
                color_frame = aligned_frames.get_color_frame() #获取RGB图
                # ir_frame_left = aligned_frames.get_infrared_frame(1)
                # ir_frame_right = aligned_frames.get_infrared_frame(2)
                if not depth_frame or not color_frame:
                    continue
                
                
                # pc.map_to(color_frame)
                # points = pc.calculate(filled_depth)
                # print(np.asarray(points.get_vertices()))
                colorizer_depth = np.asarray(depth_frame.get_data())  #colorizer.colorize(
                color_image = np.asarray(color_frame.get_data())[:,:,::-1]

 
                # import sys
                # sys.exit(0)
                # ir_left_image = np.asarray(ir_frame_left.get_data())
                # ir_right_image = np.asarray(ir_frame_right.get_data())

                self.q.put((color_image,colorizer_depth))
                # print(self.q.qsize())
            except Exception as e:
                # print(e)
                continue
        pipeline.stop()
          


class Consumer(mp.Process):
    def __init__(self,q):
        super().__init__()
        self.q = q

    def run(self):
        time.sleep(1)

        publisher = rospy.Publisher('lxz', Marker, queue_size=10)
        rospy.init_node('zhangaiwu', anonymous=True)
        rate = rospy.Rate(10)  # 10hz
        intrinsics = Box({"width":640, "height":480, "ppx":326.419, "ppy":238.673, "fx":610.51, "fy":610.54,"model":"Brown Conrady", "coeffs":[0,0,0,0,0]})
        i = 0
        depth_scale = self.q.get()
        S = Tester()
        while not rospy.is_shutdown():
            color_image, colorizer_depth = self.q.get()
            i+= 1
            if i%30 == 0:
                dst, boxes = S.infer(color_image)
                for box in boxes:
                    depth_temp = colorizer_depth[box[1]:box[3],box[0]:box[2]].astype(float)
                    depth_temp = depth_temp * depth_scale
                    dist, _, _, _ = cv2.mean(depth_temp)
                    print(dist)
                    print("Detected a {} meters away.".format(dist))

                colorizer_depth[~dst] = 0
                o3d_color = o3d.geometry.Image(color_image)  #转BGR后 open3d那颜色是对的
                o3d_depth = o3d.geometry.Image(colorizer_depth)
                rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_color, o3d_depth,
                                                                                depth_scale=1000.0, #深度范围
                                                                                depth_trunc=2.0,
                                                                                convert_rgb_to_intensity=False #彩色点云
                                                                                                                )
    
                # 转换为open3d中的相机参数
                pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
                    intrinsics.width, intrinsics.height,
                    intrinsics.fx, intrinsics.fy,
                    intrinsics.ppx, intrinsics.ppy
                )
                # 0.7.0 o3d.geometry.create_point_cloud_from_rgbd_image
                o3d_result = o3d.geometry.PointCloud.create_from_rgbd_image(
                        rgbd_image,
                        pinhole_camera_intrinsic
                    )
                o3d_result.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]) #上下翻转
                if len(o3d_result.points) > 0:
                    result = process_round2(o3d_result)
                    bbox = Box(result[0])
                    # print(time.time() - start)
                    print(result)

                    #[{'id': 'obs1', 'shape': 'BOX', 'position': {'x': 0.4024190367770307, 'y': 0.5283664877938292, 'z': 0.36600001653035474}, 'oritation': {'w': 1, 'x': -0.28480276097316193, 'y': -0.1735909525249508, 'z': -1.7859999736150107}}]
                    rospy.loginfo(result)
                    marker = aabb2marker(bbox)
                    publisher.publish(marker)
                    # rate.sleep(10)
                else:
                    rospy.loginfo("dian shu liang bu!")



def mp_run():
    mp.set_start_method(method='spawn',force=True)
    queue = mp.Queue()
    producer = Producer(q = queue, preset = 3) 
    consumer = Consumer(q = queue)
    producer.start()
    print("视频模块启动，PID号{0}".format(producer.pid))
    consumer.start()
    print("算法模块启动，PID号{0}".format(consumer.pid))
    producer.join()
    consumer.join()


if __name__ == '__main__':
    try:
        mp_run()
    except rospy.ROSInterruptException:
        pass

    '''
    速度情况下 安全距离多少 自我运算能力
    '''

