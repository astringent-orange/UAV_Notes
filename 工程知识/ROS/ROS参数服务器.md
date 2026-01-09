参数服务器通信 是ROS通信中一种比较特殊又比较简单的通信方式。参数服务器在`ROS Master`内部运行。一方向服务器设置参数，另一方则可以向服务器请求参数。
而参数服务器与`topic`与`service`不同，更加得静态，且参数服务器维护着一个数据字典，存储了参数与配置。

而与参数服务器沟通一般共有三种方式
- launch文件中设置
- 在代码中操作
- 在命令行调试

---
### 1. 在launch文件中设置
#### 1.1 写在`<node>`里面
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

#### 1.2 写在`<node>`外面（全局参数）
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

#### 1.3 加载YAML文件
批量加载参数字典
```xml
<launch>
    <!-- 加载 YAML 文件内容到参数服务器 -->
    <rosparam file="$(find my_pkg)/config/params.yaml" command="load"/>
    
    <node pkg="my_pkg" type="my_node" name="node1">
        <!-- 仅在这个节点命名空间下加载参数 -->
        <rosparam file="$(find my_pkg)/config/private_params.yaml" command="load"/>
    </node>
</launch>
```

---
### 2. 在代码中操作
#### 2.1 CPP接口
在cpp中，主要通过`ros::NodeHandle`来操作
>注意上文中提到的全局句柄和私有句柄的区别
##### 2.1.1 获取参数
**方式一：** `getParam`，先声明变量再获取
```cpp
ros::NodeHandle nh;
double radius;
// 如果参数存在，返回true并将值赋给radius；否则返回false
if (nh.getParam("/global_radius", radius)) {
    ROS_INFO("Got radius: %f", radius);
} else {
    ROS_WARN("Failed to get param 'global_radius'");
}
```
**方式二：** `param` 带有默认值
```cpp
// 语法: nh.param<类型>("参数名", 变量, 默认值);
int baud_rate;
nh.param<int>("baud_rate", baud_rate, 57600);
```

##### 2.1.2 设置与删除
```cpp
nh.setParam("new_param", "hello");
nh.deleteParam("old_param");
```
#### 2.2 Python接口
##### 2.2.1 获取参数
```python
import rospy

# 基础获取
global_radius = rospy.get_param('/global_radius')

# 带默认值的获取 (推荐)
# 如果 'baud_rate' 不存在，则返回 9600
baud = rospy.get_param('baud_rate', 9600)

# 获取私有参数 (在参数名前加 ~)
private_val = rospy.get_param('~private_param', 'default')
```

##### 2.2.2 设置参数
```python
# 设置
rospy.set_param('my_string', 'ros is cool')

# 检查是否存在
if rospy.has_param('my_string'):
    print("Param exists")

# 删除
try:
    rospy.delete_param('my_string')
except KeyError:
    print("Param not found")
```

---
### 3. 在命令行调试
- 列出所有参数
```bash
rosparam list
```
- 获取参数值
```bash
rosparam get /background_r
rosparam get / # 获取所有参数，并以YAML格式展示
```
- 设置参数值
```bash
rosparam set /background_b 100
```
- 删除参数
```bash
rosparam delete /background_b
```
- 保存/加载YAML
```bash
rosparam dump params.yaml # 导出
rosparam load params.yaml # 导入
rosparam load params.yaml /my_namespace # 导入到指定的命名空间
```

---
### 4. 参数使用例
例如在launch文件中声明
```xml
<param name="loop_rate" value="20" />
```
然后在cpp中
```cpp
int rate_hz;    // 定义变量
nh.param("loop_rate", rate_hz, 10);    // 从参数服务器获取参数
ros::Rate loop_rate(rate_hz);    // 使用参数
```