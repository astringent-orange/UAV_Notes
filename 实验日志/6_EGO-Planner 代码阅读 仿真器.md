
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

这里有一个问题，为什么要写为`simulator.xml`而不是`simulator.launch`？本质上launch文件就是xml文件，只要内容合法其运行效果是一致的。写成两种形式只是为了人为区分而形成的工程约定，即.launch文件是直接给用户运行的，而.xml文件被include的模块化启动文件