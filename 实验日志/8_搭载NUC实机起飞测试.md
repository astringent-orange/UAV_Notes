---
date: 2026-01-20
---
将NUC与摄像头粘贴到无人机上，并安装桨叶尝试运行offb文件

# 查漏补缺
首先对于里程计配置进行补充和修改，在相机外参中可以设置output文件夹，其中可以导出vins自动修正过的外参。即一开始相机的外参只需要大致正确，然后启动里程计，拿着无人机在无人机场地中缓慢地绕圈最后放回原处，然后将vins自动修正的外参粘贴至相机设置中。如此反复，直至当无人机放回原处时里程计的偏差在可以接受的范围内。

此外为了保证安全，需要通过QGC对飞控参数进行修改，对于`VEL`相关参数，设置最大速度为0.5m/s

最后，之前的offb文件也需要进行修改。因为通过设定位置来控制无人机的话，一开始无人机检测到距离目标点的差距过大，可能使用最大加速度来冲向目标点
```cpp
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    
    // 【修改1】初始发送给飞控的目标高度设为0（地面），而不是直接设为0.5
    // 这样在没解锁前，飞控收到的指令是“待在地上”
    pose.pose.position.z = 0
    
    ...
    
    ros::Time last_request = ros::Time::now();

    // 【修改2】定义起飞参数
    float target_z = 0.5;           // 最终目标高度 (米)
    float current_setpoint_z = 0.0; // 当前递增的过程高度
    float ramp_step = 0.005;        // 每次循环增加的高度 (米)
                                    // 速度 = ramp_step * rate = 0.005 * 20 = 0.1 m/s (非常安全)

    ROS_INFO("Starting Main Loop");
    
    ...
    
        // 【修改3】 柔顺起飞逻辑 (Ramp Takeoff)
        // 只有当真正进入 OFFBOARD 模式且已解锁电机后，才开始增加高度
        if(current_state.mode == "OFFBOARD" && current_state.armed){
            if(current_setpoint_z < target_z){
                current_setpoint_z += ramp_step; // 慢慢增加高度
            }
        } else {
            // 如果没解锁或者切出了Offboard，把目标高度重置为0
            // 这样下次切回来时，它会从0开始，而不是直接跳到0.5
            current_setpoint_z = 0.0;
        }

        // 将计算好的平滑高度赋值给消息
        pose.pose.position.z = current_setpoint_z;

        local_pos_pub.publish(pose); // 发送目标点位置
```

# 测试过程
直接尝试运行起飞节点，发现电机声音很大，直接使用遥控器紧急断电。推测原因一是测试空间在狭窄室内，回声较大；第二是之前没有修改起飞逻辑，电机可能直接输出最大功率

于是在修改了起飞文件后，尝试先用遥控器控制无人机起飞，观察需要多大的推力，电机的声音有多大。但是即使最大输出，无人机也无法起飞，推测是电脑含有外壳过重。拆卸外壳后尝试重新起飞，但是仍然无法起飞，发现可能桨叶受损。

检查后发现是桨叶装配方向与电机转向相反，所以推力越大反而越不能起飞。修正后尝试用遥控器起飞，发现无人机起飞十分不稳定，会左右颠簸，于是尝试调整飞控。

参考高飞自主无人机教程 https://github.com/ZJU-FAST-Lab/Fast-Drone-250/tree/master 修改部分飞控参数。此外注意到一点，碳纤维板导电，NUC拆掉外壳后不能直接接触，最好是阻隔，故重新将外壳安装。