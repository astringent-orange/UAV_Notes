---
date: 2026-01-20
---
将NUC与摄像头粘贴到无人机上，并安装桨叶尝试运行offb文件

# 查漏补缺
首先对于里程计配置进行补充和修改，在相机外参中可以设置output文件夹，其中可以导出vins自动修正过的外参。即一开始相机的外参只需要大致正确，然后启动里程计，拿着无人机在无人机场地中缓慢地绕圈最后放回原处，然后将vins自动修正的外参粘贴至相机设置中。如此反复，直至当无人机放回原处时里程计的偏差在可以接受的范围内。

此外为了保证安全，需要通过QGC对飞控参数进行修改，对于`VEL`相关参数，设置最大速度为0.5m/s

最后，之前的offb文件也需要进行修改。因为通过设定位置来控制无人机的话，一开始无人机检测到距离目标点的差距过大，可能使用最大加速度来冲向目标点。改为使用渐进的方式来起飞和降落。

```cpp
/**
 * @file offb_node.cpp
 * @brief 闭环平稳起降节点 - 分离起降拉力 + 完整调试输出
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

// 全局变量
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_local_pose;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_local_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // 订阅器
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    
    // 发布器
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    
    // 服务客户端
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0); // 必须大于 2Hz

    // 1. 等待飞控连接
    ROS_INFO("\033[1;34mConnecting to FCU...\033[0m");
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("\033[1;32mFCU Connected\033[0m");

    // 初始化设定点
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    // 2. 预发设定点（Offboard 切换前提）
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // 任务参数
    const float target_z = 1.0;      // 目标高度
    const float takeoff_dist = 0.2; // 起飞拉力距离（越大起飞越快）
    const float land_dist = 0.05;    // 降落压紧距离（越小降落越慢越稳）
    
    bool is_landing = false;
    bool timer_started = false;
    ros::Time mission_start_time;
    ros::Time last_request = ros::Time::now();
    ros::Time last_print = ros::Time::now();

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ROS_INFO("\033[1;32mStarting Main Loop\033[0m");

    while(ros::ok()){
        // A. 周期性打印当前状态 (每 4 秒)
        if (ros::Time::now() - last_print > ros::Duration(4.0)) {
             ROS_INFO("Current Mode: %s, Armed: %d, Actual_Z: %.2f", 
                      current_state.mode.c_str(), current_state.armed, current_local_pose.pose.position.z);
             last_print = ros::Time::now();
        }

        // 逻辑：如果我们已经进入了降落阶段(timer_started)，且飞控已经上锁(Armed=0)
        if(timer_started && !current_state.armed){
            ROS_INFO("\033[1;32mDisarm Detected. Mission Accomplished! Exiting...\033[0m");
            return 0; 
        }

        // B. 模式切换与解锁逻辑
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if(!is_landing){ // 降落阶段不再强切模式
                ROS_INFO("\033[1;34mAttempting to switch to OFFBOARD mode...\033[0m");
                if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                    ROS_INFO("\033[1;32mOffboard enabled\033[0m");
                } else {
                    ROS_ERROR("\033[1;31mFailed to set OFFBOARD mode!\033[0m");
                }
                last_request = ros::Time::now();
            }
        } else {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                ROS_INFO("\033[1;34mAttempting to ARM...\033[0m");
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("\033[1;32mVehicle armed\033[0m");
                } else {
                    ROS_ERROR("\033[1;31mFailed to ARM!\033[0m");
                }
                last_request = ros::Time::now();
            }
        }

        // C. 闭环跟随控制逻辑
        if(current_state.mode == "OFFBOARD" && current_state.armed){
            // 1. 起飞检测与悬停计时
            if(!timer_started && current_local_pose.pose.position.z > (target_z - 0.05)){
                mission_start_time = ros::Time::now();
                timer_started = true;
                ROS_INFO("\033[1;32mReached target altitude. Hovering for 10 seconds...\033[0m");
            }

            // 2. 悬停结束判断
            if(timer_started && !is_landing && (ros::Time::now() - mission_start_time > ros::Duration(10.0))){
                is_landing = true;
                ROS_INFO("\033[1;32mHovering finished. Initiating slow landing...\033[1;32m");
            }

            // 3. 计算设定点 Z 轴
            if(!is_landing){
                // --- 起飞阶段使用 takeoff_dist ---
                float next_z = current_local_pose.pose.position.z + takeoff_dist;
                if(next_z > target_z) next_z = target_z;
                pose.pose.position.z = next_z;
            } 
            else {
                // --- 降落阶段使用较小的 land_dist ---
                float next_z = current_local_pose.pose.position.z - land_dist;
                if(next_z < 0.05) next_z = 0; // 压向地面
                pose.pose.position.z = next_z;

                // 4. 落地切换逻辑：当高度低于 0.2m 时，调用 PX4 自带的降落模式
                if(current_local_pose.pose.position.z < 0.20 && current_state.mode != "AUTO.LAND"){
                    if(ros::Time::now() - last_request > ros::Duration(2.0)){ // 2秒冷却
                        mavros_msgs::SetMode land_set_mode;
                        land_set_mode.request.custom_mode = "AUTO.LAND";
                        if(set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent){
                            ROS_WARN("\033[1;34mSwitching to AUTO.LAND...\033[0m");
                        }
                        last_request = ros::Time::now();
                    }
                }
            }
        } else {
            // 如果不在 Offboard 或没解锁，设定高度归零
            pose.pose.position.z = 0;
        }

        // D. 核心发布：始终发送位置指令，维持心跳
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
```
然后最好先在仿真环境中运行节点。但要注意，仿真环境中能运行是最低要求，要放到真机运行可能还需要调整参数。

# 测试过程
## 阶段一
直接尝试运行起飞节点，发现电机声音很大，直接使用遥控器紧急断电。推测原因一是测试空间在狭窄室内，回声较大；第二是之前没有修改起飞逻辑，电机可能直接输出最大功率

于是在修改了起飞文件后，尝试先用遥控器控制无人机起飞，观察需要多大的推力，电机的声音有多大。但是即使最大输出，无人机也无法起飞，推测是电脑含有外壳过重。拆卸外壳后尝试重新起飞，但是仍然无法起飞，发现可能桨叶受损。

检查后发现是桨叶装配方向与电机转向相反，所以推力越大反而越不能起飞。修正后尝试用遥控器起飞，发现无人机起飞十分不稳定，会左右颠簸，于是尝试调整飞控。

参考高飞自主无人机教程 https://github.com/ZJU-FAST-Lab/Fast-Drone-250/tree/master 修改部分飞控参数。此外注意到一点，碳纤维板导电，NUC拆掉外壳后不能直接接触，最好是阻隔，故重新将外壳安装。

## 阶段二
再次尝试用遥控器起飞无人机，结果发现起飞后无人机会直给自己“摔”一圈，怀疑是电机推力不平衡。结果检查后发现一件之前没有注意过的事：在QGC中设置传感器旋转方向为180°并不能将飞行方向调整180°；即事实上，无人机的飞行方向由电机的连线决定（物理方向），当前结构下，1号电机一定在无人机飞行方向的右前方；而传感器中所设置的方向是飞控安装角度（飞控方向）与物理方向的差值；而最终摄像头的角度只能遵守物理方向。故当该角度设定有问题时，例如无人机一旦想要增大右侧电机推力，但接受的是左侧电机的数据，就会一直增大，导致瞬时翻滚。

发现px4飞控与电调之间有一个可以调整电机序号的接口，尝试更改序号后修改电机转向。结果使用dshot修改电机转向存在问题，无法保存。而电机转向是根据与电调的连线（焊接）决定的，不好修改。因此只能在既有的电机转向上修改电机的序号。


## 阶段三
起飞节点尝试切换无人机飞行模型，但是一直是ALTCTL，且查看`/mavros/local_position/pose`发现其中xy恒等于0，然后z轴发散，但是里程计读数正常。于是可知问题出现在飞控的EKF2数据融合。

使用QGC检查EKF2相关参数，发现是EKF2_AID_MASK（使用什么数据进行融合）值为1，选择的是GPS而不是vision。修改EKF2_AID_MASK=28（使用视觉、yaw、imu），EKF2_HDG_MODE（初始位置信息源）=vision（不使用气压计）。

再次运行，发现里程计正常，且local position的xyz轴都有正常读数，但是z轴初始在负值（-1.7）。需检查HDG_MODE是否还是气压计。

更改为vision后再次运行，发现z轴读数只有厘米级误差，显示正常。但是当多次用一个launch文件启动所有节点时，发现读数有时会发散。查阅后得知，launch文件不保证启动顺序，于是vins可能在摄像头之前启动，导致读数异常。改为用bash脚本启动多个launch文件，每个launch文件最好只负责一个功能，且内部没有先后顺序。
```bash
#!/bin/bash

# 1.启动mavros
echo -e "\e[1;34m[INFO] Activating Mavros...\033[0m"
roslaunch mavros px4.launch gcs_url:=udp://@192.168.8.50 &

# 检测飞控是否连接
until rostopic echo /mavros/state -n 1 2>/dev/null | grep -q "connected: True"; do
    echo -e "\e[1;34m[INFO] Waiting for FCU connection...\e[0m"
    sleep 1
done
echo -e "\e[1;32m[INFO] FCU connected!\e[0m"

# 2.启动相机
echo -e "\e[1;34m[INFO] Starting camera...\e[0m"
roslaunch vins d435_run.launch &

# 检测相机是否启动
until rostopic echo /camera/infra1/image_rect_raw -n 1 > /dev/null 2>&1;do
    echo -e "\e[1;34m[INFO] Waiting for camera stream...\e[0m"
    sleep 1
done
echo -e "\e[1;32m[INFO] Camera stream detected!\e[0m"

# 3.启动vins
echo -e "\e[1;34m[INFO] Starting VINS-Fusion...\e[0m"
roslaunch vins vins_run.launch &

# 检测vins是否启动
until rostopic echo /vins_node/odometry -n 1 > /dev/null 2>&1;do
    echo -e "\e[1;34m[INFO] Waiting for VINS to initialize...\e[0m"
    sleep 1
done
echo -e "\e[1;32m[INFO] VINS initialized!Odometry is ready\e[0m"

# 4.启动转换节点
echo -e "\e[1;34m[INFO] Starting transfer node...\e[0m"
rosrun offb vins2mavros.py &

# 检测转换节点是否启动
until rostopic echo /mavros/vision_pose/pose -n 1 >/dev/null 2>&1;do
    echo -e "\e[1;34m[INFO] Waiting for transfer node to publish data...\e[0m"
    sleep 1
done

# 5.设置伪原点
echo -e "\e[1;34m[INFO] Setting origin node\e[0m"
rosrun offb set_origin.py &

# 6.重启EKF2
echo -e "\e[1;34m[INFO] Resetting PX4 EKF2 estimator...\e[0m"
rosservice call /mavros/cmd/command "{command: 241, param1: 1, param2: 0, param3: 0, param4: 0, param5: 0, param6: 0, param7: 0}"

echo -e "\e[1;32m[INFO] All system started,ready to fly!\e[0m"

# 保持脚本不退出
wait
```

抱着无人机进行绕圈检查里程计误差。发现里程计数据误差小，但是local position误差大，于是在QGC中减小`EKF2_EVP_NOISE` `EKF2_EVV_NOISE` `EKF2_EVA_NOISE`，让飞控更相信视觉信息。同样将`EKF2_EV_DELAY`也缩小，因为是有线连接，所以可以在20~50ms。

此外，修改`EKF2_MAG_TYPE`=0，`SYS_HAS_MAG`=0，不在室内环境启用磁力计

最后尝试启动offb_node节点，观察到无人机成功切换模式并解锁，但是解锁后没有起飞。发现当目标位置太低（当前高度+0.15）时在起飞之前无人机就会上锁，修改到+0.2后成功起飞并降落。

观察到无人机的水平方向上基本没有位移，且起飞降落非常稳定；而降落时目标点为-0.05稍微有些慢，但是不影响。但是最后切换到`AUTO.LAND`后



