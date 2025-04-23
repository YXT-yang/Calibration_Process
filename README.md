# Calibration_Process
本方案中的标定流程主要适用没有实现传感器间时间硬同步的场景，按照流程可以实现**相机内参**、**IMU内参**、**IMU-相机外参**、**IMU-激光外参**的获取，外参包含了**时延**参数。
## 1. 相机内参
### 1.1 相机内参标定工具
[kalibr](https://github.com/ethz-asl/kalibr)
### 1.2 相机内参模型
可以选择的[标定模型](https://github.com/ethz-asl/kalibr/wiki/supported-models)：
普通相机模型：pinhole+radtan
鱼眼相机模型：pinhole+equi ； omni+radtan
### 1.3 制作标定板（四月格）
可以通过生成或下载的方式得到标定板文件，在打印标定板时，推荐尽量打印的大一些（>1m），标定板长期及重复使用不推荐打印纸张质或海报。
#### 1.3.1 生成标定板
在安装kalibr后，可以通过ros命令生成标定板。
```shell
# 生成aprilgrid四月格
rosrun kalibr kalibr_create_target_pdf --type apriltag --nx 6 --ny 6 --tsize 0.088 --tspace 0.3
```
```shell
# 生成checkerboard棋盘格
rosrun kalibr kalibr_create_target_pdf --type checkerboard --nx 6 --ny 6 --csx 0.03 --csy 0.02
```
#### 1.3.2 下载标定板（四月格）
可以下载[kalibr](https://github.com/ethz-asl/kalibr)给出的[标定板](https://drive.google.com/file/d/14dY7z8pDb2iEBdveTviDXsoi5H9AaQP1/view)。

![image](https://github.com/user-attachments/assets/425864b1-3693-42ea-a938-ac8bc9243850)

### 1.4 准备标定板参数文件（四月格）
在标定前需要准备标定板的参数文件，以下载的参数文件为例子，这里将标定板参数文件命名为：april_6x6_80x80cm.yaml，参数文件内容如下：
```yaml
target_type: 'aprilgrid' #gridtype
tagCols: 6               #number of apriltags
tagRows: 6               #number of apriltags
tagSize: 0.088           #size of apriltag, edge to edge [m]
tagSpacing: 0.3          #ratio of space between tags to tagSize
codeOffset: 0            #code offset for the first tag in the aprilboard
```
各参数的示意图如下，**即便生成时设置了尺寸，建议使用前用游标卡尺测量tagSize**。

![image](https://github.com/user-attachments/assets/2e17ddfe-bbfd-49d2-8daf-ce5579ec96f1)

如果准备其他格式标定板，可以[kalibr](https://github.com/ethz-asl/kalibr)中给出的[说明信息](https://github.com/ethz-asl/kalibr/wiki/calibration-targets)。
### 1.5 采集标定数据
采集标定数据时，保持平缓，记录为bag文件，在**采集数据时保证完整棋盘格一直在视野范围内**。将图像划分为如下9个区域，要保证每个区域都存在过不同倾斜方式的棋盘格。

![image](https://github.com/user-attachments/assets/a37e342e-87e2-4fe0-bc47-2805a308d422)

### 1.6 修改数据频率
[kalibr](https://github.com/ethz-asl/kalibr)官方推荐数据采集频率为4Hz，假设原始bag包为image_initial.bag，原始图像topic为/image/color，可以通过如下命令得到频率为4Hz的camera_calibration.bag（topic为/cam）：
```shell
#!/bin/bash
gnome-terminal -t "1" -x bash -c "rosbag play image_initial.bag; exec bash;"
sleep 1
gnome-terminal -t "2" -x bash -c "rosrun topic_tools throttle messages /image/color 4.0 /cam; exec bash;"
sleep 1
gnome-terminal -t "3" -x bash -c "rosbag record -O camera_calibration /cam; exec bash;"
```
### 1.7 执行相机内参标定
在做完上述准备工作后，执行下述命令即可：
```shell
rosrun kalibr kalibr_calibrate_cameras --bag camera_calibration.bag --topics /cam --models pinhole-radtan --target aprilgrid.yaml --bag-from-to 5 45
```
上述命令中，--bag-from-to后两个数值是希望使用的数据开始和结束时间，单位为秒（s）。
## 2. IMU内参标定
### 2.1 IMU内参标定工具
[imu_utils](https://github.com/gaowenliang/imu_utils)
该工具使用Allan Variance标定IMU陀螺仪及加速度计的白噪声、随机游走。环境配置方法参考该项目中[readme.md](https://github.com/gaowenliang/imu_utils/blob/master/README.md)，依赖ceres。
### 2.2 采集标定数据
按照[imu_utils](https://github.com/gaowenliang/imu_utils)中的要求，静置IMU，采集（>2h）大于两小时数据。
### 2.3 配置launch文件
在配置[launch](https://github.com/gaowenliang/imu_utils/blob/master/launch/xsens.launch)文件时，参考项目中方式即可：
```launch
<launch>
    <node pkg="imu_utils" type="imu_an" name="imu_an" output="screen">
        <param name="imu_topic" type="string" value= "/imu/data"/>		#话题名称
        <param name="imu_name" type="string" value= "xsens"/>		#IMU类型
        <param name="data_save_path" type="string" value= "$(find imu_utils)/data/"/>		#结果存放位置
        <param name="max_time_min" type="int" value= "120"/>   #使用数据长度，单位分钟（min）
        <param name="max_cluster" type="int" value= "100"/>		#最大聚合因子，一般不需要修改
    </node>
</launch>
```
在配置文件中，一般修改imu_topic和imu_name和max_time_min三个参数即可。
### 2.4 计算IMU内参
首先执行命令：
```shell
roslaunch imu_utils xsens.launch
```
当出现**wait for imu data**提示后，指定200倍播放速度播发采集的bag文件，执行命令:
```shell
rosbag play -r 200 imu_xsens.bag
```
数据播发完毕后需要等待一会才能完成解算，解算完成后查看**data_save_path**路径下**IMU名_imu_param.yaml**文件，即可得知IMU的内参。
## 3. IMU-相机外参标定
### 3.1 IMU-相机外参标定工具
[kalibr](https://github.com/ethz-asl/kalibr)
使用到了位置B-spline辅助传感器时间对齐。
### 3.2 采集标定数据
在采集标定数据时，保持运动平缓（避免碰撞、图像模糊），需要赋予三个轴向充分的激励，可以参考[kalibr](https://github.com/ethz-asl/kalibr)给出的视频：

https://www.youtube.com/watch?app=desktop&v=puNXsnrYWTY

如果没有标定经验可以沿各方向绘制几个“8”字，保证运动激励，记录得到bag文件
