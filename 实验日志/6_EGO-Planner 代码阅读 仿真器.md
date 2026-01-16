
首先在仿真中运行，是通过planner下plan_manage的launch文件进行的
![[ego 仿真启动位置.png]]
分析该launch文件的结构
```xml
1. 设定地图大小
2. 设置里程计话题
3. 启动规划器，并设置参数
   <include file="$(find ego_planner)/launch/advanced_param.xml">
4. 启动轨迹服务器
     <node pkg="ego_planner" name="traj_server" type="traj_server" output="screen">
     <node pkg="waypoint_generator" name="waypoint_generator" type="waypoint_generator" output="screen">
5. 启动仿真器
     <include file="$(find ego_planner)/launch/simulator.xml">
```

**xml文件的意义**
这里有一个问题，为什么要写为`simulator.xml`而不是`simulator.launch`？本质上launch文件就是xml文件，只要内容合法其运行效果是一致的。写成两种形式只是为了人为区分而形成的工程约定，即.launch文件是直接给用户运行的，而.xml文件被include的模块化启动文件


**traj server的作用**
planner会输出一条轨迹，但是飞控不会“执行一条轨迹”，它们只会在一个时刻接受一个setpoint，所以这里必须有一个东西来从轨迹里算出来位置、速度、加速度等信息，并以固定的频率发布


仿真器的结构
```scss
            ┌─────────────┐
            │   地图生成   │
            │ (mockamap)   │
            └─────┬───────┘
                  │ global map
        ┌─────────▼─────────┐
        │   传感器仿真层     │
        │ (pcl_render_node) │
        └─────┬───────┬─────┘
              │depth   │cloud
              │        │
      ┌───────▼──────┐ │
      │   Planner     │ │
      └───────┬──────┘ │
              │ traj    │
        ┌─────▼────────▼─────┐
        │   控制器仿真层       │
        │   (so3_control)     │
        └─────┬──────────────┘
              │ so3_cmd
        ┌─────▼──────────────┐
        │   动力学仿真层       │
        │ quadrotor_simulator │
        └─────┬──────────────┘
              │ odom
              ▼
            RViz
```
地图生成在启动时生成一份 静态3障碍物点云
而仿真传感器，作用是回答 
>如果我在某个位姿，用一台深度相机看这个世界，会看到什么？

输入：全局点云，飞机当前位置与姿态
返回：图像

内部逻辑
- 把全局点云变换到相机坐标系
- 按视锥裁剪
- 投影到图像平面

控制器仿真，相当于理想的飞控，作用是
>给出一个期望状态，给出对应的推力等输入


动力学仿真，作用是回答
>给我一个控制输入，我在物理上会怎么运动

输入：推力+姿态
内部：动力学模型，积分加速度->速度->位置
输出：里程计


画面是怎么来的？
rviz将话题数据转换为图像

上升到一个通用仿真器的结构
① 世界层（World）
地图
障碍物
地形

② 传感器层（Sensors）
相机
激光雷达
IMU

③ 动力学层（Dynamics）
刚体模型
碰撞
运动学约束

④ 控制层（Control）
PID / SO3 / MPC
理想 or 非理想

⑤ 可视化层（Visualization）
RViz / GUI / viewer

Q traj server是否应该也在simulator里？
A traj server就是应该在外面。因为不论是在仿真还是现实，都是需要traj server的