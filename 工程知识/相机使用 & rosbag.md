

# SDK启动
通过realsense自带的工具启动相机。下载教程https://blog.csdn.net/weixin_51612528/article/details/140945640
启动GUI界面`realsense-viewer`可以进行录制与查看录制结果。

## 录制视频
注意相机录制的文件格式是`.bag`，包含了除相机图像以外其他传感器数据，且所有数据都是原始格式，所以比进行了压缩的mp4文件大很多。但是直接存储原始格式对于cpu的运载压力，相较于压缩为mp4格式更小，所以推荐是录制为bag文件，等结束飞行后再转换为mp4文件。bag文件只能用`realsense-viewer`进行播放。

>注意，在飞行/录制结束后不能马上断电，因为bag文件的存储需要一段时间

ros转bag脚本[【ROS学习】 rosbag 转化为 mp4 格式视频_.bag文件怎么转成mp4-CSDN博客](https://blog.csdn.net/qq_39779233/article/details/127389801)

## 命令行模式
**基本使用方法**
`rs-record -f <video_name>.bag`持续录制至`ctrl+c`结束
`rs-record -d <time> -f <video_name>.bag`录制指定时长

**进阶使用方法**
要完成关闭结构光、只录制双目文件等录制操作，需要使用python运行脚本

# ROS启动

启动相机
使用`roslaunch realsense2_camera rs_camera.launch`启动相机

在电脑上查看
使用`rqt_image_view`可以查看图像
如果没有，则可以先下载`ros-noetic-rqt-image-view`

如果想在另一台电脑查看
则首先在NUC终端中`export ROS_MASTER_URI=http://本机ip`，`export ROS_IP=本机ip`（分别是声明master节点在哪里，以及自己是谁）
然后在另一台安装了rqt-image-view的电脑中`export ROS_MASTER_URI=http://master ip`，`export ROS_IP=本机ip`，`rqt-image-view`

## 录制视频
>首先，虽然使用ros进行录制和使用上面SDK进行录制得到的数据格式，都是`bag`，但是两者是有更细微的差距的，还是建议分开对待。

使用`rosbag`命令来进行录制与回放

**基本操作**
`rosbag record <topic_name>`录制指定的话题，生成文件名为时间戳
`rosbag record -a`代表录制所有话题
`rosbag record -O <bag_name>`指定输出的文件名（-O或者-o都可以）

`rosbag info <bag_name>`输出bag信息

`rosbag play <bag_name>`播放bag

`<node pkg="rosbag" type="record" name="bag_record" args="/topic1 /topic2"/> `在launch文件中运行

**运行流程**
首先需要启动roscore与相机（启动相机的roslaunch文件也会自动启动roscore），其次再启动录制。

**扩展**
其他类型的数据，可以用`rqt_plot`或者`plotjuggler`等gui来观察


**参考文档**
[ROS 机器人技术 - rosbag 详细使用教程！ - 知乎](https://zhuanlan.zhihu.com/p/151444739)
[ROS——一文读懂：rosbag-CSDN博客](https://blog.csdn.net/weixin_42905141/article/details/100057323)
[rosbag/Commandline - ROS Wiki](https://wiki.ros.org/rosbag/Commandline)



