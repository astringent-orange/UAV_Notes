
简单来说，三者都是cpp的编译工具。
首先，因为ros项目非常依赖本地环境且要进行自主修改，所以基本都需要在本地对源码进行编译。（拓展：对于一个cpp的应用程序，其是否需要上述内容？并不需要，一个应用程序可以以编译后的二进制形式发布。一种方式是静态连接，将依赖抄进exe文件；第二种方式是动态链接，例如使用win的dll，在运行时编译，这样只需要一个安装程序，在安装时将这些文件放到指定文件夹中）

# cmake
对于简单的cpp文件编译，例如只有两个文件，可以用命令行手动编译
```bash
g++ main.cpp lib.cpp -o final
```
但是实际中，ros项目的可能有几十上百个需编译的文件，分布在不同文件夹中，且有极多路径不同的依赖和头文件。每次手动敲指令编译太过困难。

因而有了`makefile`文本文件，将编译指令输进去，之后每次输入make即可执行编译过程。
但是`makefile`也有问题。例如不同平台的文件路径不同，相同的依赖可能在不同的路径中，且每次新加一个文件就需要手动修改`makefile`

于是有了cmake工具，根据电脑环境自动生成`makefile`。相当于多了一个中间层，使用者通过更高层的`CMakeList.txt`去自动生成更底层的makefile。cmake有几大特点：首先会自动寻找依赖，第二可以跨平台使用，第三其通过让顶层cmakelist指挥底层cmakelist可以管理超大项目。

# catkin make
可以视为cmake的高阶封装版，ROS1自带的编译工具，用于高效地构建众多相互依赖但独立开发的CMake项目。有一个workspace，将ws下视为一个巨大的cmake项目。

缺点在于没有实现包的隔离，会导致变量名冲突，以及修改一个包需要重新检查整个工作空间的依赖关系


# catkin build
由`catkin_tools`提供的更现代ROS1的编译工具，需要自己下载。
`sudo apt-get install python3-catkin-tools`
可以实现多个包并行的单独编译，提供单独的build文件夹和日志等，有更好的输出结果展示。

## catkin config
在使用`catkin init`初始化工作空间后，可以对工作空间进行一些配置，以优化操作

`catkin config --extend /opt/ros/$ROS_DISTRO`

`catkin config --merge-devel`
合并编译结果的空间。本来catkin build默认每一个功能包都有单独的devel空间，从而可以单独删除某一个包的编译结果，但是这样python的依赖路径可能很深。

`catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release`
给底层的编译器传递参数。默认模式为debug或空，而`Release`模式会开启编译器优化（-O3），代码运行速度比 Debug 模式快数倍

`catkin config --cmake-args -DCMAKE_CXX_FLAGS=-fdiagnostics-color`
让编译器输出的错误信息带颜色。

`catkin config --install`
安装模式，在编译完成之后会生成一个可执行的二进制包，这样可以直接发送到服务器上并用docker执行，避免了巨大的源码拷贝

`catkin config --blacklist/whitelist bag_name`
黑/白名单，不编译/只编译某个包

`catkin config -j/--jobs number`
手动限制并行编译的线程数，默认编译会占满所有资源




**参考文档**
[catkin config – 配置工作空间 — catkin_tools 0.0.0 文档](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_config.html)

# 扩展
catkin工具都是ROS1专用的，即仅能用于ROS项目（其他工具无法识别package.xml文件），且只适用于1代。

ROS2则使用`colcon`，在继承了catkin build的并行隔离编译的基础上，还支持多种语言的项目

而其他大型cpp项目，一般可以用`Ninja+CMake`进行编译


**cmakelist多个文件编译为一个节点**
单独编译每一个文件，然后将各个文件链接在一起。那么为什么要写成三个文件？第一是提高编译速度（增量编译），如果全部写在一个文件中，那么即使有一行代码改变那么整个文件都要重新编译；而分为多个文件，则只用重新编译那个修改的文件。第二是为了模块化与可读性，实现逻辑分离。第三是为了代码复用。第四是为了多人协作。