import open3d as o3d
import numpy as np

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0, 1, 0])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

def process_round2(pcdfile:o3d.geometry.PointCloud):
    aabb_list, obscales = [], []
 
    downpcd = pcdfile.voxel_down_sample(voxel_size=0.01)
    print(len(downpcd.points))
    _ , ind = downpcd.remove_statistical_outlier(nb_neighbors=200, std_ratio=0.5)
    in_downpcd = downpcd.select_by_index(ind)

    labels = np.array(in_downpcd.cluster_dbscan(eps=0.25, min_points=10, print_progress=False))
    # eps： 定义到聚类相邻点云的距离
    # min_points： 定义形成聚类所需的最小点数
    max_label = labels.max() + 1#最大值相当于共有多少个类别
    for label in range(0, max_label):
        label_index = np.where(labels == label)  # 提取分类为label的聚类点云下标
        label_pcd = in_downpcd.select_by_index(np.array(label_index)[0])  # 根据下标提取点云点
        # print('label: ', str(label), '点云数：', len(label_pcd.points))
        # 点云AABB包围盒
        aabb = label_pcd.get_axis_aligned_bounding_box()
        aabb.color = (0, 1, 0)
        aabb_list.append(aabb)
        # print(np.asarray(aabb.get_box_points())) #输出8个点坐标
        center = aabb.get_center()#输出中心点
        xyz = aabb.get_extent() #输出XYZ 轴长度

        obs = "obs{}".format(label+1)
        obscale={
              "id": obs,
              "shape": "BOX",
              "position": {
                "x": xyz[0],
                "y": xyz[1],
                "z": xyz[2] #?
              },
              "oritation": {
                "w": 1,
                "x": center[0],
                "y": center[1],
                "z": center[2]
              }
                }
        obscales.append(obscale)
          

            # 可视化
    #show_ops = [in_downpcd]+[i for i in aabb_list]
    #o3d.visualization.draw_geometries(show_ops, window_name=obs,
    #                                width=800,  # 窗口宽度
    #                                height=600)
    return obscales

if __name__ == "__main__":
    # 加载点云
    downpcd = o3d.io.read_point_cloud("1.pcd")
    # print(len(pcd.points))
    # voxel_size = 0.01
    # downpcd = pcd.voxel_down_sample(voxel_size)
    # print(len(downpcd.points))
    aabb_list = []

    #离群点剔除
    # print("Statistical oulier removal")
    _ , ind = downpcd.remove_statistical_outlier(nb_neighbors=200, std_ratio=0.5)
    #nb_neighbors 允许指定要考虑多少个邻居，以便计算给定点的平均距离
    #std_ratio 允许基于跨点云的平均距离的标准偏差来设置阈值级别。此数字越低，过滤器将越具有攻击性
    # display_inlier_outlier(downpcd, ind)
    in_downpcd = downpcd.select_by_index(ind)


    labels = np.array(in_downpcd.cluster_dbscan(eps=0.25, min_points=10, print_progress=False))
    # eps： 定义到聚类相邻点云的距离
    # min_points： 定义形成聚类所需的最小点数
    max_label = labels.max() + 1 
    for label in range(0, max_label):
        label_index = np.where(labels == label)  # 提取分类为label的聚类点云下标
        label_pcd = in_downpcd.select_by_index(np.array(label_index)[0])  # 根据下标提取点云点
        print('label: ', str(label), '点云数：', len(label_pcd.points))
          # 点云AABB包围盒
        aabb = label_pcd.get_axis_aligned_bounding_box()
        aabb.color = (0, 1, 0)
        aabb_list.append(aabb)
        # print(np.asarray(aabb.get_box_points())) #输出8个点坐标
        center = aabb.get_center()#输出中心点
        xyz = aabb.get_extent() #输出XYZ 轴长度


        obscales={
              "id": "obs1",
              "shape": "BOX",
              "position": {
                "x": xyz[0],
                "y": xyz[1],
                "z": xyz[2]
              },
              "oritation": {
                "w": 1,
                "x": center[0],
                "y": center[1],
                "z": center[2]
              }
                }
          

        print(obscales)
      #


          # 可视化
    show_ops = [in_downpcd]+[i for i in aabb_list]
    o3d.visualization.draw_geometries(show_ops, window_name="Object:Box"+str(label),
                                    width=800,  # 窗口宽度
                                    height=600)




    # print(labels.shape)
    # max_label = labels.max() #最大值相当于共有多少个类别
    # print(f"point cloud has {max_label + 1} clusters")

    # colors = np.random.randint(255, size=(max_label+1, 3))/255.
    # print(colors)
    # colors = colors[labels]

    # print(in_downpcd[labels])
    # in_downpcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    # # o3d.visualization.draw_geometries([in_downpcd],
                                    
    # #                                 window_name="Object:Box",width=800,  # 窗口宽度
    # #                                   height=600)




  


    # 点云OBB包围盒
    # obb = pcd.get_oriented_bounding_box()
    # obb.color = (0, 1, 0)
    # 可视化滤波结果
    # o3d.visualization.draw_geometries([in_downpcd,aabb], window_name="Object:Box",
    #                                   width=800,  # 窗口宽度
    #                                   height=600)



