---
date: 2026-01-05
---

### 文件结构
![[ego 代码文件结构.png]]
planner为规划器相关，而uav_simulator为在模拟时的仿真环境相关。在planner下，bspline为优化函数相关，pathsearching为A* 相关，plan_env为建图相关，plan_manage为整个项目的入口（其下src中有一个ego_planner_node文件，其中有main函数），traj为轨迹相关代码

使用plan manage的launch文件中的run in sim启动文件，会调用advanced param.xml文件中的高级参数设置

### 运行逻辑
#### 初始化
run in sim.launch逻辑，运行 advanced param.xml（只要看开头与结尾是launch标签，那么xml文件就可以作为launch文件运行；这样写是为了层级化和结构化），运行ego planner节点，运行simulator仿真器。——>关键在于第二步ego planner节点。查看cmake文件可以知道，ego planner节点是由三个文件拼接而成
![[ego planner node拼接.png]]
其中，ego planner node cpp中包含了main函数，为整体入口
``` cpp
using namespace ego_planner;

int main(int argc, char **argv)

{
  ros::init(argc, argv, "ego_planner_node");
  ros::NodeHandle nh("~");

  EGOReplanFSM rebo_replan;
  
  rebo_replan.init(nh);

  ros::Duration(1.0).sleep();
  ros::spin();
  
  return 0;
}
```
在main中，首先初始化了一个节点，然后声明了一个`EGOReplanFSM`类实体（FSM表示finite state machine有限状态机），然后运行其初始化函数。发现该类是一个在自定义库文件`ego_replan_fsm.h`中的类，但是其`init`函数则是在`ego_replan_fsm.cpp`中定义，于是查看`init`的内容。
发现首先是设定了句柄的参数，其次初始化了一些模组，最后设置了回调函数，于是在去寻找设定的模组与回调，首先是模组。
```cpp
    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
```
去寻找其最初被声明的位置，发现是在类的私有元素中。
```cpp
    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    ego_planner::DataDisp data_disp_;
```
听起来很绕，先解析一下语法，理清楚关键字的作用，文件的结构，起作用的顺序，以`planner_manager_`为例。

首先是扩起来的作用域 `namespace ego_planner`，增加一个变量的作用域，防止变量重名，即如果在不同的作用域下要使用`EGOPlannerManager`类，则需要使用完整名`ego_planner::EGOPlannerManager`，而在`fsm.h`中也使用了同样的作用域，则不必使用完整名。
其次，和类名同名的函数表示构造函数与析构函数（与python的init函数区分，这里的init函数是一个自己定义的函数）。
第三，在`EGOPlannerManager`类成员中写了两次public，语法上是没有问题，只是不太规范。
```cpp
class EGOPlannerManager{
  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;
}
```
第四，关于Ptr的使用，在`EGOPlannerManager`类中有这样的定义`typedef unique_ptr<EGOPlannerManager> Ptr;`，首先要清楚这是定义了一个类型的别名，即这是一个类型成员而不是对象成员，而在cpp中对于类型成员使用`ClassName::Type`来进行访问（类型成员和函数则用点进行分隔）。其次，什么是`unique_ptr`？这是cpp11中引入的智能指针（区别于裸指针），可以自动删除指向的对象（而不必手动delete），其次同一时间只会有一个指针指向被管理的对象，其作用基本与裸指针相同。此处相当于定义了一个指向类自身的指针类型。
第五，关于`reset`的使用。首先`reset`是智能指针的成员函数，使用方式如`p.reset(q) //q为智能指针要指向的新对象`，其实此处就是为指针分配空间。于是梳理一下整个流程：`main`函数声明了一个`FSM`类的实例，该类中有两个指向成员类的指针，调用`init`函数为这两个成员类分配空间

**为什么如此设计？**
为什么要绕一大圈用指针来表示类成员呢？而不是作为直接类成员？有以下几点原因。
首先是控制初始化时机，使用指针之后可以控制类成员的初始化时间，例如设计在ros节点句柄nh准备好之后。
其次是可扩展性，使用指针那么之后只需要更改类中的内容而不必更改变量名。

#### 主循环
依旧是在`init`函数中开启主循环，

`nh.createTimer`的作用。和之前基础内容中的循环类似，之前是用rate和while中的sleep来控制循环的时间，定时器也是用于循环，但是是一种更安全的循环
```cpp
ros::Rate r(10); // 10 hz
while (ros::ok())
{
  ... do some work ...
  r.sleep();
}
```
首先定时器的定义方式如下
```cpp
ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);
```
而官方代码中的形式为
```cpp
 exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
```
其中回调函数的部分有些不同，这里使用的是一个成员函数指针，而this则代表使用当前实例的函数。Q：如果实际执行时间超出设定的间隔会怎样？A：只会错过运行周期，接着继续运行但是不会崩溃.
而对于回调函数的形式基本固定如下
```cpp
void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
{
	...
}
```
其参数内容为固定写法，e包含上一次与这一次回调函数应该和实际发生的时间
Q：两个timer之间的关系如何？A：回调函数相当于向队列中加入了一个任务，然后由main中的spin逐个取出然后执行；两个timer会共享一个任务队列，同样由spin逐个执行，相当于串行执行。

每一次执行`exec_timer`，会根据当前的状态（一个枚举类型变量）进行切换行为，相当于一个有限状态机。
```cpp
switch (fsm_state_)
{
    case INIT:
        // 初始化状态
        break;
    case WAIT_TARGET:
        // 等待目标
        break;
    case GEN_NEW_TRAJ:
        // 生成新轨迹
        break;
    case EXEC_TRAJ:
        // 执行轨迹
        break;
    case REPLAN_TRAJ:
        // 重新规划轨迹
        break;
}
```

#### 感知-建图-规划-优化如何体现



#### 代码使用问题 
**Eigen库**
cpp中的线性代数库，使用该库可以完成例如矩阵乘法等运算。其中根据头文件不同，其导入方式也不一致。
```cpp
- #include <Eigen/Core> //只引入最基础的矩阵和数组功能（编译快）。
- #include <Eigen/Dense> //引入所有稠密矩阵运算（最常用）。
- #include <Eigen/Eigen> //引入所有模块（包括稀疏矩阵 Sparse、几何模块 Geometry 等）。
```
因为此处用了许多高级功能，故直接导入所有模块。而编译器默认是找不到 `<Eigen/Eigen>` 在哪里的，因为它通常安装在 `/usr/include/eigen3` 这种非标准路径下。需要告诉 CMake 去哪里找。通常在camkelist中有如下部分
```cmake
# 1. 查找 Eigen 库包
find_package(Eigen3 REQUIRED)

# 2. 将 Eigen 的头文件路径加入到编译器的搜索路径中
# 这步之后，编译器才能看懂 <Eigen/Eigen> 到底在硬盘的哪个位置
include_directories(
    ${EIGEN3_INCLUDE_DIRS}
)

# 3. 编译节点 (回顾刚才学的)
add_executable(ego_planner_node src/ego_planner_node.cpp)

# 4. 链接库
# 注意：Eigen 是 "Header-only" (纯头文件) 库！
# 它通常不需要 target_link_libraries 链接 .so 文件，
# 只要 include 了头文件，代码其实就已经包含进去了。
# (不过为了保险或某些特定模块，有时也会写在链接里，或者只链接 catkin_LIBRARIES)
```


