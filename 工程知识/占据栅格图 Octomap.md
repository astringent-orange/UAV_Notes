
参考 [ROS中的三维地图表示：OctoMap详解及应用-CSDN博客](https://blog.csdn.net/m0_73640344/article/details/144833619)
在传统分阶段（感知-建图-规划-优化）任务中，环境建图是非常重要的一环。`octomap`是一个开源的三维环境建模库，其基于八叉树`octree`实现，可以将三维**点云**数据转换为三维地图

以Ubuntu 20.04 + ROS noetic为例，安装过程如下
```bash
sudo apt-get update
sudo apt-get install ros-noetic-octomap ros-noetic-octomap-mapping ros-noetic-octomap-server
# rviz可视化插件
sudo apt-get install ros-noetic-octomap-rviz-plugins
# 离线查看工具，可直接查看地图.bt文件
sudo apt-get install octovis
```

>确保机器人有能提供三维点云数据的传感器

`octomap`在使用时是作为一个ROS节点而存在，其输入为点云+定位信息，输出占据栅格+可视化标记。最基本的启动方式为`roslaunch octomap_server octomap_mapping.launch`，可以启动rviz添加`octomap`，选择`/octomap_full`进行查看

一般可以通过launch文件启动，便于配置参数
```xml
<launch>
  <node pkg="octomap_server" type="octomap_server_node" name="octomap_server">
    
    <!-- ================= 核心坐标系 ================= -->
    <!-- frame_id: 世界坐标系 (地图的根)，通常是 VINS 输出的 "world" 或 "map" -->
    <param name="frame_id" type="string" value="world" />
    
    <!-- base_frame_id: 机器人的基座，通常是 "body_link" 或 "base_link" -->
    <!-- 用于滤除地面或机器人自身的点云 (可选) -->
    <param name="base_frame_id" type="string" value="body_link" />

    <!-- ================= 建图参数 ================= -->
    <!-- resolution: 格子大小 (米)。0.05 代表 5cm -->
    <param name="resolution" value="0.05" />

    <!-- max_range: 传感器最大有效距离 -->
    <!-- D435 建议设为 4.0 - 5.0。超过这个距离的点云不可信，不用于建图 -->
    <param name="sensor_model/max_range" value="5.0" />
    
    <!-- ================= 动态更新逻辑 ================= -->
    <!-- latch: false 表示地图会实时刷新。如果门开了，旧的障碍物会消失。 -->
    <!-- latch: true 表示只增不减。适合完全静态的环境。 -->
    <param name="latch" value="false" /> 

    <!-- 概率参数 (通常用默认即可) -->
    <!-- hit: 击中障碍物增加的概率; miss: 光线穿过减少的概率 -->
    <param name="sensor_model/hit" value="0.7" />
    <param name="sensor_model/miss" value="0.4" />
    
    <!-- ================= 地面滤除 (可选) ================= -->
    <!-- 如果不想要把地板建成障碍物，开启 filter_ground -->
    <param name="filter_ground" value="false" />

    <!-- ================= 数据输入 ================= -->
    <!-- cloud_in: 你的点云话题名字 -->
    <remap from="cloud_in" to="/camera/depth/color/points" />
  
  </node>
</launch>
```

---
### 使用D435 的点云
