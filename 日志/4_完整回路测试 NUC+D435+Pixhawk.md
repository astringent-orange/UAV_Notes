---
date: 2025-12-25
tags:
  - MAVROS
  - px4
  - vins
  - offb_node
---

### 实验设想
在由Intel NUC13ANKi7-1360P + Intel Realsense D435 + Pixhawk（固件版本1.11.0） 组成的完整真机环境中，运行VINS_Fusion（使用相机信息与飞控IMU），且通过里程计设定无人机位置，运行基本测试节点（包含飞控切换模式，解锁，设定目标点等功能），预计观察到无人机（卸下桨叶）电机旋转。

基本节点offb_node 实现飞控切换模式，解锁然后设定预期高度


---
### 一、构建项目空间
**1. 构建工作空间**
``` bash
mkdir -p offb_ws/src
cd offb_ws/src
catkin_make
```

**2. 创建里程计节点**
(1) 下载里程计文件
因为此处测试不需要完整的里程计文件，且为了减少文件包的嵌套，单独将其中的vins包与camera包拿出使用
``` bash
git clone https://github.com/Geneta2580/Geneta_vins_fusion ./temp
mv ./temp/vins_estimator ./vins_estimator
mv ./temp/camera_models ./camera_models
rm -rf ./temp
# 检查当前文件夹/offb_ws/src
# 应该显示camera_models vins_estimator CMakeList.txt
ls
# 成功后先编译里程计文件
cd ~/offb_ws/src/
catkin_make
```

(2) 创建vins配置文件

(3) 创建相机launch文件

(4) 创建里程计launch文件

**3. 创建运动节点**
单独放在另外一个功能包中
``` bash
cd ~/offb_ws/src
catkin_create_pkg offb roscpp rospy geometry_msgs mavros_msgs
cd offb/src
```
添加如下测试节点文件`offb_node.cpp`
``` cpp
/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;   // 无人机当前状态
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
  
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    ROS_INFO("Connecting FCU");
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU Connected");

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
  
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
  
    ROS_INFO("Starting Main Loop");
    while(ros::ok()){
    // 每隔1秒打印一次无人机状态
    static ros::Time last_print = ros::Time::now();
    if (ros::Time::now() - last_print > ros::Duration(1.0)) {  
	    ROS_INFO("Current Mode: %s, Armed: %d",  current_state.mode.c_str(), current_state.armed); 
	    last_print = ros::Time::now(); 
	}
    
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            // 切换 Offboard 模式
            ROS_INFO("Attempting to switch to OFFBOARD mode...");
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            else{
	            ROS_ERROR("Failed to set OFFBOARD mode!");
            }

            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                //无人机解锁
                ROS_INFO("Attempting to ARM...");
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                else{
	                ROS_ERROR("Failed to ARM!");
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub.publish(pose); // 发送目标点位置
  
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
```
修改`CMakeList.txt` `vim ~/offb_ws/src/offb/CMakeList.txt`在末尾添加以下内容
``` bash
add_executable(offb_node src/offb_node.cpp)
target_link_libraries(offb_node ${catkin_LIBRARIES})
```

**4. 编译文件**
``` bash
cd ~/offb_ws/src
catkin_make
```

---
### 二、里程计配置
**1. 准备IMU数据**
连接飞控启动mavros，运行`rostopic hz /mavros/imu/data_raw`查看imu频率，要求在200hz左右，如果在50hz左右，则参考以下方法。
(1) 启动mavros时不去设置后续参数，例如使用
`roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0` 去掉后续的`:57600`

(2) 或者使用QGC连接飞控，将mavlink模式设置为`onboard`。

首先如果是电脑远程登陆的NUC，可以使用`roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0 gcs_url:=udp://@192.168.1.2`来将数据转发给电脑（最后的ip为电脑的ip）。通常打开QGC会自动连接，如果没有则点击左上角图标->application settings->comm links，add->Name:NUC_LINK Type，Port:14550（默认端口）->OK，然后选择connect。

然后左上角图标->vehicle setup->parameters；在搜索框搜索MAV_，确保MAV_0_CONFIG为TELEM 1 或 Auto；修改MAV_0_MODE，从normal改为onboard（也可能是MAV_1或者MAV_2）；点击右上角tools，重启飞行器。

ps：onboard和offboard并不是相反的作用。其中onboard代表mavlink模式设置，代表了数据传输速率，有normal与onboard，前者为窄带宽的无线传输模式，后者为高带宽的有线模式。而offboard代表了飞行模式，代表谁在控制飞机，通常有manual代表人使用遥控器操控，mission代表飞控按照预设航点操控，offboard代表nuc操控。

(3) 如果前两种方法不管用，则可以修改飞控SD卡，使用脚本强行修改频率。使用读卡器将飞控sd卡连接电脑，在根目录下创建`etc`文件夹，然后建立一个文本文件`extras.txt`，在其中写入如下内容
``` bash
mavlink start -d /dev/ttyACM0 -b 921600
mavlink stream -d /dev/ttyACM0 -s ATTITUDE_QUATERNION -r 200
mavlink stream -d /dev/ttyACM0 -s HIGHRES_IMU -r 200
```
保存后插回即可。

可能的问题：
(1) 文档格式不正确。首先确认文件名称为`extras.txt`而不是`extras.txt.txt`，如果是在win上可以点击“查看”显示文件扩展名来确认。其次，如果是在win上配置的该文件，可能存在换行符问题，即win与unix的换行符不同导致飞控无法识别。需要使用vscode打开文件，确认右下角是LF而不是CRLF。
(2) 文档内容不正确。如果在其中写了`mavlink stop-all -d /dev/ttyACM0`可能导致启动mavros时一直卡住。
(3) sd卡问题。px4飞控对于sd卡的设置有着严格要求，要求sd卡格式化为FAT32格式，且簇大小为32K，分区表使用MBR。如果使用win对sd卡进行格式化，win会默认使用GPT分区表。其次，win默认无法对于超过32G的sd卡格式化为FAT32（可以用命令行实现，但是非常慢，且可能出错），建议使用diskgenius [DiskGenius Download Center | Free Download DiskGenius](https://www.diskgenius.com/download.php)。


**2. 修改VINS配置文件**
参考vins_fusion测试一节，但是其中需要修改一部分内容，让vins使用飞控的imu数据（之前没有使用imu）。
(1) 开启使用imu数据
(2) 设置imu话题
(3) imu参数设置
(4) 相机与imu相对参数设置
``` yaml
# 仅列出部分需要修改的地方
# 首先是打开imu的使用并配置话题
imu: 1
imu_topic: "/mavros/imu/data_raw"

# 其次是设定相机与imu（假设是飞控的中心）的相对位置
# 如果相机没有进行旋转，面向飞行方向，则只需要修改前三行最后一个数字
# 分别代表相机在xyz（前左上）相对于imu的偏移，单位为米
body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [  0,  0,  1.0,  0.04,
           -1.0,  0,  0,  0.01,
            0,  -1.0, 0,  0.04,
            0,  0,  0,  1.0]

body_T_cam1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [  0,  0,  1.0,  0.04,
           -1.0,  0,  0,  -0.02,
            0,  -1.0,  0,  0.04,
            0,  0,  0,  1.0]

# 开启在线优化
estimate_extrinsic: 1

# imu参数设置。因为使用飞控的imu，其噪声较大，所以需要修改默认值
acc_n: 0.2          # accelerometer measurement noise standard deviation. #0.2   0.04
gyr_n: 0.05         # gyroscope measurement noise standard deviation.     #0.05  0.004
acc_w: 0.002         # accelerometer bias random work noise standard deviation.  #0.002
gyr_w: 0.0003       # gyroscope bias random work noise standard deviation.     #4.0e-5
```


---
### 三、位姿回传
vins输出的消息话题以及内容与px4飞控要求的不一致，所以需要一个转换节点。具体而言vins输出的是`/vins_node/odometry`，而px4要求`/mavros/vision_pose/pose`。使用python在offb功能包中完成，`vim offb_ws/src/offb/scripts/vins2mavros.py`（python节点不用编辑cmake文件）

``` python
#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def callback(msg):
    # 将 VINS 的 Odometry 转换为 PX4 需要的 PoseStamped
    p = PoseStamped()
    p.header.stamp = rospy.Time.now()
    p.header.frame_id = "map" # 或 odom
    p.pose = msg.pose.pose
    # 注意：这里省略了坐标系旋转（ENU vs NED），如果只是原地测试电机，通常没问题。
    # 如果实飞，必须使用坐标转换库（如 tf）进行严格对齐。
    pub.publish(p)

rospy.init_node('vins_to_mavros_transfer')
pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
sub = rospy.Subscriber('/vins_estimator/odometry', Odometry, callback)
rospy.spin()
```


---
### 四、PX4参数设置
**1. QGC配置参数**
在nuc上启动mavros并连接QGC对飞控的参数进行设置。
(1) 设置`EKF2_AID_MASK`为24
(2) 设置`EKF2_HGT_MODE`为3
(3) 设置`EKF2_EV_NOISE_MD`为`EV noise parameters`

**2. 阶段性实验**
检测里程计是否正常运行，读数是否没有较大误差。

存在问题：
(1) 里程计发散
(2) 里程计计数变为0
正常来说，在目前没有引入误差消解的情况下，运动1m左右有厘米级的误差是正常情况。上述两个问题是由于EKF和失效重置导致的，可以按照以下方法逐步矫正。

解决方法：
(1) 调整vins配置文件中位置参数，检查单位是否是米，采用更精确的配置。注意不要使用默认的参数，那是d435i内置imu的位置。
(2) 增大imu的噪声。原有的噪声是d435i内置imu的噪声，飞控的imu噪声更大，可以不断调整尝试。
(3) 检查前进方向和数据方向是否符合。向前x增加，向左y增加，向上z增加
(4) 重新配置无人机加速度计参数。如果加速度计配置不正确，可能导致用于抵消重力加速度的加速度泄露到其他方向，造成无人机即使不动其位置读数也会增加。
(5) 刚启动里程计时不要动无人机，刚启动时里程计较为脆弱
(6) 摄像头所正对的墙壁最好不要用纯色墙壁，会影响里程计判断

---
### 五、执行实验
**1. 汇总launch文件**
(1) 汇总launch文件。将mavros启动、相机启动、vins启动、转换节点启动写在同一个launch文件中。

ps：为什么不将启动offb也写在launch文件中？因为在关闭节点时，如果将offb写在一起，会导致所有节点“同时”关闭，而飞控在offboard模式下失去信息来源将会报警。正确做法是先切换模式，再关闭节点。

**2. 启动节点**
将无人机连接QGC，在执行完上述launch文件之后，正常来说正上方应该显示绿色的ready to fly


存在问题：
(1) 报错：PositionTargetGlobal failed because no origin。说明飞控没有接受到`/mavros/global_position/gp_origin`话题的内容来初始化位置
解决方法：
<1> 连接QGC修改参数
①遥控信号丢失保护 (COM_RCL_ACT 或 NAV_RCL_ACT)
默认值：Return (返航) —— 这就是报错的元凶。
修改为：Hold mode (悬停/1) 或 Land mode (降落/0)。
说明：室内没有 GPS，绝对不能选 Return。
②Offboard 控制丢失保护 (COM_OBL_ACT)
默认值：Land 或 Return。
修改为：Land mode (降落) 或 Hold mode (悬停)。
说明：当你的 offb_node 挂了或数据发慢了，飞控该怎么办。
③数据链丢失保护 (NAV_DLL_ACT)
默认值：Hold 或 Return。
修改为：Hold mode (悬停) 或 Disabled。
④无 GPS 解锁限制 (COM_ARM_WO_GPS)
在 v1.11 版本中，通常默认允许无 GPS 解锁，但最好检查一下。
确保设置为 1 (Allow)。

<2> 设置一个欺骗节点，手动发布初始化位置
``` bash
cd ~/offb_ws/src/offb/scripts
vim set_origin.py
sudo chmod +x set_origin.py
```

``` python
#!/usr/bin/env python3
import rospy
from geographic_msgs.msg import GeoPointStamped

# 这是一个欺骗节点的脚本
# 它告诉飞控：我们当前的全球坐标原点在哪里
# 这样飞控就不会报 "No global origin" 的错误了

def set_fake_origin():
    rospy.init_node('set_fake_origin_node', anonymous=True)
    
    # 发布到 MAVROS 的设置原点话题
    pub = rospy.Publisher('/mavros/global_position/set_gp_origin', GeoPointStamped, queue_size=10)
    
    rospy.loginfo("Setting fake global origin...")
    
    # 等待发布者建立连接
    rospy.sleep(2)
    
    # 构建一个假的 GPS 点
    # 室内飞行只看相对位置
    p = GeoPointStamped()
    p.header.stamp = rospy.Time.now()
    p.header.frame_id = "map"
    p.position.latitude = 0.0      # 随便填
    p.position.longitude = 0.0    # 随便填
    p.position.altitude = 0.0
    
    # 持续发送几次，确保飞控收到
    for i in range(5):
        pub.publish(p)
        rospy.sleep(1)
        
    rospy.loginfo("Fake origin set commands sent.")

if __name__ == '__main__':
    try:
        set_fake_origin()
    except rospy.ROSInterruptException:
        pass
```

(2) 报错：FCU: Primary compass not found
室内环境与机载电脑会对磁力计产生巨大影响，需要通过QGC关闭磁力计的使用，修改无人机参数
设置`EKF2_MAG_TYPE`为0；设置`SYS_HAS_MAG`为0


