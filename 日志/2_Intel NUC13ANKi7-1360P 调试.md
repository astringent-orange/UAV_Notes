---
date: 2025-12-23
tags:
  - NUC
  - wifi
---

20251223 Intel NUC13ANKi7-1360P安装过程记录
**1.  安装Ubuntu 20.04**
(1) 烧录镜像安装盘，ubuntu 20.04 amd64 桌面版
(2) 安装镜像：连接显示器，键盘，鼠标以及无线网卡
存在问题：
<1>显卡驱动未安装导致分辨率异常，无法看到完整的安装界面，从而不能点击进行下一步：
① 尝试直接拖动窗口顶部； 
② 或alt+f7+鼠标左键直接拖动窗口本身；
③ 或按win键打开搜索栏寻找setting，找到display选项修改分辨率；
④ 或ctrl+alt+t 调出终端 `xrandr`查看分辨率相关信息 `xrandr -s 1920x1080` 修改分辨率

(3) 完成安装，下载如`openssh`等必要软件
``` bash
# 下载openssh
sudo apt install openssh-server -y

# 检查服务状态，若为绿色的active则表示已启动
sudo systemctl status ssh

# 允许防火墙通过
sudo ufw allow ssh
```

存在问题
<1> 系统安装过程中安装了libreoffice以及媒体等不必要软件：
① 卸载libreoffice `sudo apt remove --purge libreoffice* -y`
② 卸载邮件客户端 `sudo apt remove --purge thunderbird -y`
③ 卸载播放器与小游戏 `sudo apt remove --purge rhythmbox totem aisleriot gnome-mahjongg gnome-mines gnome-sudoku transmission-common simple-scan -y`
④ 卸载应用商店 `sudo snap remove snap-store`
⑤ 自动清理 `sudo apt autoremove -y`

(4) 额外设置。可关闭图形化界面以提升性能
``` bash
# 关闭图形化界面
sudo systemctl set-default multi-user.target

# 重启生效
sudo reboot

# 如果想要打开图形化界面
sudo systemctl set-default graphical.target
sudo reboot
```

---
**2. 基础调试**
(1) 连接网络。
存在问题：
<1> 安装完成后发现有线网络与蓝牙都可以正常工作，但是找不到wifi图标，系统设置中也没有wifi选项；`ip addr` 也找不到无线接口。
原因：
① 主板开启了scure boot模式
② 内核断层：NUC13的网卡所采用协议，对于ubuntu 20.04默认的kernel 5.15过高=>将内核升级到6.x版本
③ 固件断层：内核升级为6.x版本后请求的系统微码版本为77-89，而Ubuntu 20.04 自带库中最高只有71

解决方法：
① 通过网络稳定的电脑与NUC直连，作为文件传输中转站
② 安装兼容旧系统的现代化内核：
在网络稳定电脑上进入网站 https://github.com/wkennington/linux-firmware 下载zip包后解压，然后传输给nuc，如`scp -r D:\Downloads\linux-firmware-master drone@192.168.55.100:/tmp/`，登陆NUC然后操作
``` bash
cd /tmp/linux-firmware-master

# 覆盖固件
sudo cp iwlwifi-* /lib/firmware/
sudo cp intel/ibt-* /lib/firmware/intel/

# 重启
sudo reboot

# 检查
uname -r
# 应该出现如6.12.63
```


③ 注入缺失固件
根据 `dmseg | grep -i iwl` 的提示报错，如`Direct firmware load for iwlwifi-so-a0-gf-a0-83.ucode failed with error -2`，下载特定微码文件并覆盖。
在网络稳定主机上进入linux kernel官方仓库 https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/tree/ 。根据缺失的文件搜索如``
`iwlwifi-so-a0-gf-a0`的目标文件，选择数字合适的文件。点击进入该文件，再点击plain下载。此外还需下载`iwlwifi-so-a0-gf-a0.pnvm`文件。
将文件传输至NUC，如`scp iwlwifi-so-a0-gf-a0* drone@192.168.55.100:/tmp/` ，然后登陆NUC后进行操作
``` bash
# 移动文件 
cd /tmp 
sudo cp iwlwifi-so-a0-gf-a0* /lib/firmware/ 
# 再次确认文件已就位 
ls -l /lib/firmware/iwlwifi-so-a0-gf-a0* 
# 确保刚才传进来的 -83.ucode 在里面

# 重启nuc
sudo reboot

# 检查
ip addr
```

普遍性分析：
该情况非常普遍，任何在2023/2024年后发布的硬件上安装Ubuntu 18.04/20.04 的行为都可能出现该情况

---
**3. 飞行控制**
(1) 安装ROS1
``` bash
# 添加ros软件源
1. sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   
# 添加ros密钥
1. wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
   
# 安装
sudo apt update
sudo apt install ros-noetic-desktop-full
```
设置环境变量，打开`~/.bashrc`文件，`sudo vim ~/.bashrc`
在文件末尾添加`source /opt/ros/noetic/setup.bash`
关闭文件后刷新 `source ~/.bashrc`
测试是否成功`roscore`

(2) 安装mavros
``` bash
# 安装mavors二进制包
sudo apt update
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras -y

# 安装地理数据
# 下载安装脚本
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh

# 赋予执行权限并运行（需要下载几十 MB 数据，可能需要一点时间）
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

# 配置串口权限
sudo usermod -a -G dialout $USER
sudo reboot
```

至此机载电脑完成基本调试