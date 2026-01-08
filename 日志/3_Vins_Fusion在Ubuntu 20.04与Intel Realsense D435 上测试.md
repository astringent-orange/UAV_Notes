---
date: 2025-12-22
tags:
  - vins
  - d435
  - Ubuntu20
---

### 注意事项
**1.vins适配版本**
首先要注意原项目只支持ubuntu 16.04与18.04，在20.04 及以上版本因为默认的opencv库从opencv3升级为4，导致许多库文件不再适配（部分变量名称改变），强行使用编译时会报错，需要自己修改库文件
https://github.com/HKUST-Aerial-Robotics/VINS-Fusion

Ubuntu 20.04支持版
https://github.com/Geneta2580/Geneta_vins_fusion
本次使用20.04适配版，该版本中已经默认修改了库文件，可以直接编译。

**2.硬件准备**
(1) 本次使用intel realsense d435，而非d435i，vins项目对后者提供了默认的配置文件，而前者需要在其基础上修改
(2) 为了传输相机的视频数据，需要使用usb3.0支持，即要求用支持usb3.0的线缆将相机接在nuc的usb3.0接口。（ps：可以观察usb接口来判断是否为usb3.0。usb3.0接口通常为蓝色胶芯，且除了4个较长的黄色金属弹片外还有5个较短的弹片）

---
### 1. 安装Realsense驱动与ROS接口
- **SDK (Librealsense)**：这是底层驱动，负责通过 USB 协议与摄像头硬件通信，获取原始数据流。即通过代码调用该库向相机发送命令。
- **ROS Wrapper**：这是一个“翻译官”。它将 SDK 获取的原始数据封装成 ROS 格式的消息（Topic），让 VINS 能够订阅和读取。可以通过指令`roslaunch realsense2_camera rs_camera.launch` 启动了一个C++程序，调用SDK初始化硬件，然后开启一个死循环：不断从 USB 接口读取图像数据 -> 转换成 ROS 格式的消息 (`sensor_msgs/Image`) -> 发布到话题 (`Topic`) 上。
``` bash
# 1. 更新软件源
sudo apt-get update

# 2. 安装 RealSense 的 ROS 驱动包 (包含 SDK 和 ROS 节点)
# 相比源码编译，直接用 apt 安装最稳定，不容易出现内核版本冲突
sudo apt-get install ros-noetic-realsense2-camera ros-noetic-realsense2-description
```

---
### 2. 安装Ceres Solver优化库

- **VINS 的本质**：VINS 是一个巨大的数学优化问题（非线性最小二乘问题）。它在不断计算：“根据我看到的图像变化，我移动了多少，才能让误差最小？”
- **Ceres Solver**：这是 Google 开发的数学库，专门用来解这种复杂的方程。VINS 的核心算法严重依赖它。
``` bash
# Ubuntu 20.04 的软件源里已经包含了 Ceres，直接安装开发库即可
sudo apt-get install libceres-dev libgoogle-glog-dev
```

---
### 3. 下载并编译VINS_Fusion
按照官方readme文档进行操作
``` bash
# 首先创建一个工作空间
mkdir -p vins_ws/src & cd vins_ws/src

# 下载vins_fusion
git clone https://github.com/Geneta2580/Geneta_vins_fusion

# 编译代码
cd ..
catkin_make
source ./devel/setup.bash
```

存在问题
(1) 如电脑配置过低，如内存大小<2G，编译时可能因内存空间不足停止。可以使用`catkin_make -j1`来编译，不过可能导致编译速度过慢
(2) 如果强行在Ubuntu 20.04上编译原版的vins_fusion，可能会出现两种报错导致编译停止。都是因为Ubuntu 自带的opencv版本改变了，Ubuntu 18.04为opencv3，Ubuntu 20.04 为opencv4

<1> 变量名因版本升级改变。
报错示例：
``` bash
/home/sunrise/UAV/catkin_ws_vins/src/VINS-Fusion/vins_estimator/src/KITTIOdomTest.cpp:95:39: error: ‘CV_LOAD_IMAGE_GRAYSCALE’ was not declared in this scope  
95 | imLeft = cv::imread(leftImagePath, CV_LOAD_IMAGE_GRAYSCALE );  
| ^~~~~~~~~~~~~~~~~~~~~~~
```
打开报错的文件`vim ~/UAV/catkin_ws_vins/src/VINS-Fusion/vins_estimator/src/KITTIOdomTest.cpp` 将其中所有的`CV_LOAD_IMAGE_GRAYSCALE` 替换为 `cv::IMREAD_GRAYSCALE`

<2> 缺少头文件
报错示例：
``` bash
/home/sunrise/UAV/catkin_ws_vins/src/VINS-Fusion/loop_fusion/src/ThirdParty/DVision/BRIEF.cpp: In member function ‘void DVision::BRIEF::compute(const cv::Mat&, const std::vector[cv::KeyPoint]&, std::vector<boost::dynamic_bitset<> >&, bool) const’:  
/home/sunrise/UAV/catkin_ws_vins/src/VINS-Fusion/loop_fusion/src/ThirdParty/DVision/BRIEF.cpp:53:32: error: ‘CV_RGB2GRAY’ was not declared in this scope  
53 | cv::cvtColor(image, aux, CV_RGB2GRAY);  
| ^~~~~~~~~~~
```
打开报错的文件`vim ~/UAV/catkin_ws_vins/src/VINS-Fusion/loop_fusion/src/ThirdParty/DVision/BRIEF.cpp`
在开头添加以下两行
``` cpp
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>
```

---
### 4. 修改vins配置文件
编译完成后需要自己手动编写.yaml的配置文件。vins的官方代码中规定了需要.yaml文件，且写死了其中一些变量的名称，但是.yaml文件的位置则并不固定，可以作为main函数参数传入。虽然配置文件的位置没有规定，但是一般放在config文件夹中
而vins官方代码为d435i提供了默认的配置文件，可以直接运行。而本次使用d435，需要在d435i配置文件的基础上进行一定的修改方可运行。

``` bash
# 进入项目默认的d435i配置文件库，复制一份d435i配置文件并修改
cd ~/UAV/vins_ws/src/Geneta_vins_fusion/config/realsense_d435i
cp realsense_stereo_imu_config.yaml d435_stereo_no_imu.yaml
vim d435_stere0_no_imu.yaml
```
修改配置文件的部分参数
``` yaml
%YAML:1.0

# --- 1. IMU 设置 ---
imu: 0         # 【重要】改成 0，表示不使用 IMU
# imu_topic: "/camera/imu"  # 注释掉或留着都没事，反正上面关了

# --- 2. 图像 Topic 设置 ---
# 确保和 roslaunch 启动的话题一致
image0_topic: "/camera/infra1/image_rect_raw"
image1_topic: "/camera/infra2/image_rect_raw"

# --- 3. 相机参数 ---
model_type: PINHOLE
camera_name: camera
image_width: 640
image_height: 480
# 这里的内参(fx, fy, cx, cy)最好后续做一次标定，目前先用文件里默认的测试

# --- 4. 外参 (Extrinsic) ---
# 因为没有 IMU，相机到 IMU 的外参变得不重要，但为了防止报错，
# 建议保持默认或设为 estimate_extrinsic: 0
```


---
### 5. 运行与测试
**(1) 命令行启动相机**
``` bash
# 最短命令 roslaunch realsense_camera rs_camera.launch
roslaunch realsense2_camera rs_camera.launch \
  enable_infra1:=true \
  enable_infra2:=true \
  infra_rgb:=false \
  enable_gyro:=false \
  enable_accel:=false \
  emitter_enabled:=false \
  enable_sync:=true \
  unite_imu_method:=none \
  depth_width:=640 \
  depth_height:=480 \
  depth_fps:=30
```
正常启动会提示`RealSense Node Is Up! `

存在问题
<1> 出现异常`fail to open usb interface: 0, error: RS2_USB_STATUS_ACCESS`。解决方案参考CSDN https://blog.csdn.net/qq_32618327/article/details/120730198 。

**(2) 启动vins**
根据vins官方readme启动ros节点
``` bash
source ~/catkin_ws/devel/setup.bash
rosrun vins vins_node <path_to_your_config_file>/config.yaml
```


**(3) 检测**
``` bash
# 检查相机是否正常启动
rostopic list | grep camera
rostopic hz /camera/infra1/image_rect_raw

# 检查里程计是否正常启动
rostopic list | grep vins
rostopic echo /vins_estimator/odometry
```
正常情况下，相机话题输出频率在30hz左右，而里程计话题可能如下
``` yaml
header: 
  seq: 125
  stamp: 
    secs: 1703666666
    nsecs: 123456789
  frame_id: "world"
child_frame_id: "body"
pose: 
  pose: 
    position: 
      x: 0.01234      <--- 关注这里
      y: -0.05678     <--- 关注这里
      z: 0.10111      <--- 关注这里
    orientation: 
      x: 0.0...
      y: 0.0...
      z: 0.0...
      w: 0.9...
twist: 
  ...
```

存在问题
<1> 相机可以正常启动，但是检测话题频率为0。可能是因为使用了usb2.0的数据线与电脑接口。使用`lsusb -t` 查看`class=video`一项的最后的数据是否是480M，若是则代表了使用的是usb 2.0协议；如果是5000M则是3.0协议。尝试更换数据线与电脑接口后重新检测。

---
### 6. 进阶配置
**(1) 使用launch文件启动相机和vins节点**
将两者分别写为launch文件
``` bash
cd ~/UAV/vins_ws/src/Geneta_vins_fusion/vins_estimator/launch
vim d435_run.launch
vim vins_run.launch

cd ~/UAV/vins_ws
source ./devel/setup.bash
```

相机启动文件
``` xml
<launch>
  <!-- 定义相机名称 -->
  <arg name="camera_name"       default="camera"/>
  
  <!-- 核心配置：640x480 @ 30fps (USB 3.0 模式下最稳) -->
  <arg name="image_width"       default="640"/>
  <arg name="image_height"      default="480"/>
  <arg name="fps"               default="30"/>

  <!-- 调用官方驱动 -->
  <include file="$(find realsense2_camera)/launch/rs_camera.launch">
    <arg name="camera"          value="$(arg camera_name)"/>
    
    <!-- 1. 开启双目红外 (VINS 需要) -->
    <arg name="enable_infra"    value="true"/>
    <arg name="enable_infra1"   value="true"/>
    <arg name="enable_infra2"   value="true"/>
    <arg name="infra_rgb"       value="false"/>
    
    <!-- 2. 关闭不需要的流 (节省带宽) -->
    <arg name="enable_color"    value="false"/>
    <arg name="enable_depth"    value="false"/> <!-- 关掉深度流，减轻 CPU 负担 -->
    <arg name="enable_gyro"     value="false"/>
    <arg name="enable_accel"    value="false"/>

    <!-- 3. 设置分辨率 -->
    <arg name="infra_width"     value="$(arg image_width)"/>
    <arg name="infra_height"    value="$(arg image_height)"/>
    <arg name="infra_fps"       value="$(arg fps)"/>

    <!-- 4. 关键设置 -->
    <arg name="enable_sync"     value="true"/>  <!-- 开启硬件时间同步 -->
    <arg name="align_depth"     value="false"/>
    <arg name="initial_reset"   value="true"/>  <!-- 每次启动强制复位 USB，防止僵尸状态 -->
    
    <!-- 注意：不要在这里写 emitter_enabled，因为你的驱动版本不支持 -->
  </include>

  <!-- 5. 强行关闭结构光 (防止红点干扰 VINS) -->
  <group ns="$(arg camera_name)">
    <rosparam param="stereo_module/emitter_enabled">false</rosparam>
  </group>
</launch>
```

里程计启动文件
``` xml
<launch>
    <arg name="config_path" default="/home/drone/UAV/vins_ws/src/Geneta_vins_fusion/config/realsense_d435i/d435_stereo_no_imu.yaml" />
    <node pkg="vins" type="vins_node" name="vins_node" args="$(arg config_path)" output="screen" />
</launch>
```

存在问题：
launch文件的位置防止不正确，例如放在了`/Geneta_vins_fusion/launch/`下。如果位置不正确则尝试运行roslaunch时报错无法找到launch文件。

原因：
`Geneta_vins_fusion`文件夹并不是一个ros包，其下的`vins_estimator`和`camera_models`才是真正的ros包，判断依据是该文件夹下是否有`package.xml`文件。故ros无法将其`Geneta_vins_fusion`识别为一个ros包，更不用提其下的launch文件夹了。

解决方法：
将launch文件放在`/vins_estimator/launch`下

ps：为什么文件夹名称是`vins_estimator`但是在运行vins节点时却使用了包名为vins？因为报名并不是由文件夹名称来决定的，而是由`package.xml`文件决定，其中规定了`<name>vins</name>`。即ros是通过识别`package.xml`来识别一个ros包

**(2) 使用Rviz可视化**

**(3) 结合飞控IMU**

