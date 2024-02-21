# GCS-FixedwingSwarm 

# 固定翼无人机集群地面站

基于dronekit和pysimplegui库的轻量化固定翼无人机集群地面站，适用于运行ardupilot固件的无人机，并支持SITL仿真模式。

**20240221更新：增加了支持课题组开发的三维视景仿真系统的代码，已测试通过5v5对抗实时显示**

本项目原本是基于一个在FMS Ranger 1800mm固定翼无人机上运行的任务机代码修改的，因为老板要求参加一个破比赛，不得不对任务机程序进行修改和移植，加了一些对我来说没什么屁用的功能。

使用方法：

1.从github上clone本项目

```
git clone https://github.com/TongBingda/GCS-FixedwingSwarm.git
```

2.安装原版dronekit、matplotlib、pyserial等库

```
pip install dronekit
pip install pysimplegui
pip install pyserial
pip install matplotlib
```

3.安装经过修改的dronekit库，已经向dronekit主项目提交pull request，具体更新时间未知

```
pip uninstall dronekit
cd dronekit-python-master
python setup.py build
python setup.py install
```

4.运行RangerMission3.1.py文件，此时应该可以看到主界面了。最好使用Silicon Labs cp2102芯片的数传，其他数传没有测试过，有可能认不到串口。

有问题请发邮件至tongbingda@buaa.edu.cn，我随缘解答，随缘更新，随缘修bug。

5.20240108更新，在使用基站时，无人机上的任务机应当运行Mavproxy转发程序

```
home/bafs/.local/bin/mavproxy.py --out=tcpin:192.168.1.xxx:8080
```

地面站应当选中Simulation/Base Station选项才能正常连接