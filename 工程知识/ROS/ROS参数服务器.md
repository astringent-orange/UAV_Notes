参数服务器通信 是ROS通信中一种比较特殊又比较简单的通信方式。参数服务器在`ROS Master`内部运行。一方向服务器设置参数，另一方则可以向服务器请求参数。
而参数服务器与`topic`与`service`不同，更加得静态，且参数服务器维护着一个数据字典，存储了参数与配置。

而与参数服务器沟通一般共有三种方式
- launch文件中设置
- 在代码中读取
- 在命令行调试

#### 在launch文件中设置
##### 写在`<node>`里面
```xml
<launch>
    <!-- 参数夹在 node 标签中间 -->
    <node pkg="ego_planner" name="drone_1" type="ego_planner_node">
        <param name="fsm/flight_type" value="1" />
    </node>
</launch>
```
ros会自动给参数加上节点得前缀，即该参数在服务器中全名变为` /drone_1/fsm/flight_type`以此来防止冲突和支持多机协同

而为了读取这种“私有参数”，需要创建一个“私有句柄”
```cpp
// 这里的 "~" 表示：我要在这个节点的私有目录下找参数
ros::NodeHandle nh("~"); 
// 此时它查找的是 /node_name/fsm/flight_type
nh.param("fsm/flight_type", ...);
```

##### 写在`<node>`外面（全局参数）
```xml
<launch>
    <!-- 参数直接放在 launch 下 -->
    <param name="global_gravity" value="9.8" />

    <node pkg="ego_planner" name="drone_1" type="ego_planner_node">
    </node>
</launch>
```
同样句柄中则不需要加~
```cpp
// 这里的 nh 没有 "~"，表示它是全局或相对句柄
ros::NodeHandle nh; 
// 它会在根目录查找 /global_gravity
nh.param("global_gravity", ...);
```

如果两种参数都要使用该怎么办？通常可以声明两个句柄，一个公有句柄，一个私有句柄。
```cpp
void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    // 1. 用私有句柄读取节点专属配置 (比如 PID 参数、文件名)
    // 查找 /node_name/fsm/flight_type
    nh_private.param("fsm/flight_type", ...); 

    // 2. 用全局句柄读取公共配置 (比如 全局地图分辨率、仿真时间)
    // 查找 /global_radius (或者是相对当前命名空间的路径)
    nh.param("global_radius", ...);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_node");

    ros::NodeHandle nh;        // 全局/公共句柄
    ros::NodeHandle nh_private("~"); // 私有句柄

    init(nh, nh_private); // 把两个都传进去
}
```