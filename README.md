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

[video](https://www.youtube.com/watch?app=desktop&v=puNXsnrYWTY)

如果没有标定经验可以沿各方向绘制几个“8”字，保证运动激励，记录得到bag文件。

### 3.3 准备相关文件

在执行IMU-相机的外参标定时，需要准备IMU内参文件、相机内参文件和数据bag文件。

#### 3.3.1 IMU内参文件

IMU内参文件格式如下：

```yaml
#Accelerometers
accelerometer_noise_density: 5.43036e-03   #Noise density (continuous-time)
accelerometer_random_walk:   1.44598e-04   #Bias random walk

#Gyroscopes
gyroscope_noise_density:     4.9700e-03   #Noise density (continuous-time)
gyroscope_random_walk:       6.8522e-05   #Bias random walk

rostopic:                    /imu/data      #the IMU ROS topic
update_rate:                 100.0      #Hz (for discretization of the values above)
```

其中，**Accelerometers**和**Gyroscopes**参数，取IMU内参（[imu_utils](https://github.com/gaowenliang/imu_utils)工具）[标定结果](https://github.com/gaowenliang/imu_utils/blob/master/data/xsens_imu_param.yaml)中**avg-axis**值即可。IMU数据topic需要在本文件指定，IMU数据采集频率同样在这里指定。

#### 3.3.2 相机内参文件

相机内参文件格式如下:

```yaml
cam0:
  cam_overlaps: []
  camera_model: pinhole
  distortion_coeffs: [-0.037616380662515145, -0.01017328830844074, 0.0021156274599045564, -0.00041016163581791436]
  distortion_model: equidistant
  intrinsics: [348.9572868848721, 351.98561676978676, 356.0925444657544, 238.4353424788532]
  resolution: [720, 540]
  rostopic: /cam_1
```

该内参文件在进行相机内参标定时可以直接得到。

#### 3.3.3 bag数据文件

直接采集得到的bag文件中，图像topic的频率需要修改为4Hz。假设原始bag包为image_imu_initial.bag，原始图像topic为/image/color，原始IMU的topic为可/imu/data，以通过如下命令得到频率为4Hz的camera_calibration.bag（topic为/cam_1、/imu/data）：

```shell
#!/bin/bash
gnome-terminal -t "1" -x bash -c "rosbag play image_imu_initial.bag; exec bash;"
sleep 1
gnome-terminal -t "2" -x bash -c "rosrun topic_tools throttle messages /image/color 4.0 /cam_1; exec bash;"
sleep 1
gnome-terminal -t "3" -x bash -c "rosbag record -O camera_imu_calibration /cam_1 /imu/data; exec bash;"
```

上述命令打包为shell文件，在命令行运行。

### 3.4 执行IMU-相机外参标定

在做完上述准备工作后，执行外参标定：

```shell
rosrun kalibr kalibr_calibrate_imu_camera --target aprilgrid.yaml --bag camera_calibration.bag --bag-from-to 5 50 --cam camera.yaml --imu ium.yaml --imu-models calibrated
```

其中，**--imu-models**指定标定结果中IMU相关参数，有**calibrated**（默认模型）， **scale-misalignment**（简化的 calibrated 模型） 和，**scale-misalignment-size-effect**（加入了传感器本身体积引起的误差）三种格式，相关格式可以参考文章：

[Extending kalibr: Calibrating the extrinsics of multiple IMUs and of individual axes](https://ieeexplore.ieee.org/abstract/document/7487628)

[kalibr](https://github.com/ethz-asl/kalibr)中的[说明](https://github.com/ethz-asl/kalibr/wiki/Multi-IMU-and-IMU-intrinsic-calibration#joern1)

一般不需要进行更改。

## 4. IMU-激光外参标定

### 4.1 IMU-激光外参标定工具

[ikalibr](https://github.com/Unsigned-Long/iKalibr)

算法设置了**build_thirdparty.sh**一键式三方库安装工具，如果在配置环境时遇到任何问题，可以在**issues**中提问，都能够很快得到回复，推荐**star**一下该工作。

该[作者](https://github.com/Unsigned-Long)实现了很多种传感器之间的外参标定，包含多种相机，我之所以在这里使用[kalibr](https://github.com/ethz-asl/kalibr)标定相机参数，值是因为内参文件不需要修改，**推荐follow这个作者**。

### 4.2 采集标定数据

采集数据前不需要建立标定场，在进行数据采集时需要对IMU三个轴向都赋予充分的激励，运动时要保持平滑。

使用ikalibr进行标定时，需要选取运动部分，可以在采集数据时进行记录，也可以在采集数据后使用plotjuggler工具进行查看。需要注意，在**开始时的静止区间的数据无法使程序初始化**，仅取用运动部分即可！

### 4.3 准备相关文件

#### 4.3.1 launch文件

在[launch](https://github.com/Unsigned-Long/iKalibr/blob/master/launch/solver/ikalibr-prog.launch)文件中需要设置**config_path**为自己的config（.yaml）文件。

```launch
<?xml version="1.0" encoding="UTF-8" ?>
<launch>

    <arg name="config_path" default="$(find ikalibr)/config/ikalibr-config.yaml"/>

    <node pkg="ikalibr" type="ikalibr_prog" name="ikalibr_prog" output="screen">
        <!-- change the value of this field to the path of your self-defined config file -->
        <param name="config_path" value="$(arg config_path)" type="string"/>
    </node>
</launch>
```

#### 4.3.2 config文件

在进行IMU和激光间外参标定时，参考[config文件](https://github.com/Unsigned-Long/iKalibr/blob/master/config/ikalibr-config.yaml)如下：

```yaml
Configor:
  DataStream:
    # key: IMU topic, value: IMU type. Supported IMU types are:
    #   1. SENSOR_IMU: https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html (acce unit: m/s^2)
    #   1. SENSOR_IMU: https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html (acce unit: m/s^2)
    #   2.    SBG_IMU: https://github.com/SBG-Systems/sbg_ros_driver.git (acce unit: m/s^2)
    #   3. SENSOR_IMU_G: https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html (acce unit: G)
    IMUTopics:
      # at least one IMU is integrated in the sensor suite
      - key: "/mavros/imu/data"
        value:
          Type: "SENSOR_IMU"
          Intrinsics: "/home/whu/ThirdLib/SLAM/iKalibr/src/ikalibr/config/imu-intri.yaml"
          AcceWeight: 17.68
          GyroWeight: 57.66
      - key: "/livox/imu"
        value:
          Type: "SENSOR_IMU_G"
          Intrinsics: "/home/whu/ThirdLib/SLAM/iKalibr/src/ikalibr/config/imu-intri.yaml"
          AcceWeight: 17.68
          GyroWeight: 57.66
    RadarTopics:
    LiDARTopics:
      - key: "/livox/lidar"
        value:
          Type: "LIVOX_POINTS"
          Weight: 100.0
    CameraTopics:
    ReferIMU: "/livox/imu"
    BagPath: "Your_Data_Path/lidar_imu_calib.bag"
    BeginTime: 5
    Duration: 110
    OutputPath: "Your_Output_Path"
  Prior:
    # unit: m/s^2
    GravityNorm: 9.8
    # if sensor are hardware-synchronized, you could choose to fix temporal parameters
    # by setting this field to 'false'
    OptTemporalParams: true
    # the range where the time offsets would be optimized.
    # make sure this range contains the ground truth of time offsets
    # If you're not sure, make this field large, but this could lead to longer optimization time
    TimeOffsetPadding: 0.10
    # readout time padding for RS camera, make sure this range contains the ground truth
    ReadoutTimePadding: 0.01
    # the time distance of two neighbor control points, which determines the accuracy
    # of the representation of the B-splines. Smaller distance would lead to longer optimization time
    # common choices: from '0.01' to '0.10'
    KnotTimeDist:
      SO3Spline: 0.05
      ScaleSpline: 0.05
    # when lidar is involved in the calibration framework, the ndt odometer is employed to recover pose roughly
    NDTLiDAROdometer:
      # 0.5 for indoor case and 1.0 for outdoor case
      Resolution: 0.5
      KeyFrameDownSample: 0.1
    LiDARDataAssociate:
      # leaf size when down sample the map using 'pcl::VoxelGrid' filter
      # note that this field just for visualization, no connection with calibration
      # for outdoor, 0.1 is suggested, and for indoor: 0.05 is suggested
      MapDownSample: 0.05
      # associate point and surfel when distance is less than this value
      PointToSurfelMax: 0.1
      # chose plane as a surfel for data association when planarity is larger than this value
      PlanarityMin: 0.6
    # the loss function used for radar factor (m/s)
    CauchyLossForRadarFactor: 0.1
    # the loss function used for lidar factor (m)
    CauchyLossForLiDARFactor: 0.5
    # the loss function used for visual reprojection factor (pixel)
    CauchyLossForCameraFactor: 1.0
  Preference:
    # whether using cuda to speed up when solving least-squares problems
    # if you do not install the cuda dependency, set it to 'false'
    UseCudaInSolving: false
    # currently available output content:
    # ParamInEachIter, BSplines, LiDARMaps, VisualMaps, RadarMaps, HessianMat,
    # VisualLiDARCovisibility, VisualKinematics, ColorizedLiDARMap
    # AlignedInertialMes, VisualReprojErrors, RadarDopplerErrors
    # NONE, ALL
    Outputs:
      - LiDARMaps
      - ColorizedLiDARMap
    # supported data output format:
    # 1. JSON
    # 2. XML
    # 3. YAML
    # 4. BINARY (not recommended)
    # do not dwell on it, we have provided an additional ros program to perform data format transform
    OutputDataFormat: "YAML"
    # number of thread to use for solving, negative value means use all valid thread to perform solving
    ThreadsToUse: -1
    # scale of splines in viewer, you can also use 'a' and 'd' keys to
    # zoom out and in splines in run time
    SplineScaleInViewer: 3.0
    # scale of coordinates in viewer, you can also use 's' and 'w' keys
    # to zoom out and in coordinates in run time
    CoordSScaleInViewer: 0.3
```

在yaml文件中，传感器部分仅保留**IMUTopics**和**LiDARTopics**部分子内容即可，**IMUTopics**中的**AcceWeight**及**GyroWeight**分别由IMU内参标定结果中加速度计和陀螺仪的白噪声取倒数得到。

如：加速度计白噪声值为：5.880E-4 (m/s^2/sqrt(hz))，则权重计算值为(1.0/5.880E-4) = 1700.68，赋**AcceWeight**值为：17.68。

如：陀螺仪的白噪声值为：1.745E-4 (rad/s/sqrt(hz))，则权重计算值为(1.0/1.745E-4) = 5730.66，赋**GyroWeight**值为：57.66。

激光的权重不需要更改，样条的相关参数一般也不需要更改。

当有多个IMU时，需要注意设置**ReferIMU**，即哪个topic对应的IMU为中心。

需要注意设置**BagPath**为采集数据的路径。

需要注意设置**OutputPath**为输出结果的路径。

**BeginTime**和**Duration**的含义分别为bag数据中开始运动的时间，和所使用数据的时长，单位均为秒（s）。

**IMUTopics**下**Intrinsics**对应的[IMU内参](https://github.com/Unsigned-Long/iKalibr/blob/master/config/imu-intri.yaml)数据可以通过IMU-相机外参标定结果中IMU的参数设置，一般情况下不进行设置不影响标定，[ikalibr](https://github.com/Unsigned-Long/iKalibr)会给出一套标定结果。

### 4.4 执行IMU-激光外参标定

在设置完上面相关文件后，执行外参标定命令：

```yaml
roslaunch ikalibr ikalibr-prog.launch
```

如果运动激励充分，等待一会即可得到标定结果。

## 5 感谢开源工作者的贡献！
