# IMU-internal parameters

因为在标定IMU内参的过程中遇到了数据单位的问题，在查看[imu_utils0](https://github.com/gaowenliang/imu_utils)的issue的过程中，我发现有人同样提出了类似的问题，并给出了自己所撰写的代码[imu_utils1](https://github.com/mintar/imu_utils.git)，现在我通过解算同一组数据，验证一下[imu_utils0](https://github.com/gaowenliang/imu_utils)解算得到的参数的单位。

此外，我会使用[kalibr_allan](https://github.com/rpng/kalibr_allan)的解算结果作为参考值。

## 1.环境搭建

### [imu_utils0](https://github.com/gaowenliang/imu_utils)环境

搭建环境时，首先执行如下操作：

```shell
mkdir imu_utils0 & cd imu_utils0
mkdir src & cd src
git clone https://github.com/gaowenliang/imu_utils.git
git clone https://github.com/gaowenliang/code_utils.git
```

解下来，将原始[code_utils](https://github.com/gaowenliang/code_utils)中[CMakeLists.txt](https://github.com/gaowenliang/code_utils/blob/master/CMakeLists.txt)修改为如下形式：

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(code_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC -fopenmp")

find_package(catkin REQUIRED
    roscpp
    std_msgs
    )

#set(EIGEN_INCLUDE_DIR "/usr/local/include/eigen3")
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES polynomial cv_utils pnp
    CATKIN_DEPENDS roscpp std_msgs
#    DEPENDS system_lib
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
     )
include_directories("include")

add_library(polynomial STATIC
    src/math_utils/Polynomial.cpp)
target_link_libraries(polynomial ${Boost_LIBRARIES} )

add_library(pnp
    src/cv_utils/dlt/dlt.cpp
    src/cv_utils/pnp/pnp.cpp
    src/cv_utils/pnp/linearpnp.cpp
    src/cv_utils/pnp/nonlinearpnp.cpp)
target_link_libraries(pnp ${Boost_LIBRARIES}  ${OpenCV_LIBS}  ${CERES_LIBRARIES})

add_library(cv_utils STATIC
    src/cv_utils.cc
    )
target_link_libraries(cv_utils ${Boost_LIBRARIES}  ${OpenCV_LIBS} )

# add_executable(matIO_test   src/mat_io_test.cpp )
# target_link_libraries(matIO_test dw ${OpenCV_LIBS})

# add_executable(sumpixel_test   src/sumpixel_test.cpp )
# target_link_libraries(sumpixel_test dw ${OpenCV_LIBS})
```

同时，将[imu_utils0](https://github.com/gaowenliang/imu_utils)中[package.xml](https://github.com/gaowenliang/imu_utils/blob/master/package.xml)修改为如下形式：

```xml
<?xml version="1.0"?>
<package format="2">
  <name>imu_utils</name>
  <version>0.1.0</version>
  <description>The imu_utils package</description>
  <maintainer email="gaowenliang@todo.todo">gaowenliang</maintainer>
  <license>MIT</license>
  <author >gaowenliang</author>
  <depend>code_utils</depend>
  <buildtool_depend>catkin</buildtool_depend>
  <export>
  </export>
</package>
```

接下来，从`src`文件夹退出到`imu_utils0`文件夹，并执行编译命令：

```shell
cd ..
catkin_make -j -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

### [imu_utils1](https://github.com/mintar/imu_utils.git)环境

在搭建[imu_utils1](https://github.com/mintar/imu_utils.git)时方法同上。

### [kalibr_allan](https://github.com/rpng/kalibr_allan)环境

搭建环境时，首先执行如下操作：

```shell
mkdir kalibr_allan & cd kalibr_allan
mkdir src & cd src
git clone https://github.com/rpng/kalibr_allan.git
```

然后将`bagconvert/cmake`子文件夹下[FindMatlab.cmake](https://github.com/rpng/kalibr_allan/blob/master/bagconvert/cmake/FindMatlab.cmake)删除。

并将`bagconvert`子文件夹下[CMakeLists.txt](https://github.com/rpng/kalibr_allan/blob/master/bagconvert/CMakeLists.txt)文件修改为如下形式，其中`MATLAB_DIR`需要设置为你的安装位置：

```cmake
cmake_minimum_required(VERSION 2.8.3)

# Project name
project(bagconvert)

# Include our cmake files
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)

# Include libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
set(MATLAB_DIR "/usr/local/MATLAB/R2023a")
set(MATLAB_INCLUDE_DIR
    ${MATLAB_DIR}/extern/include
)
set(Matlab_LIB_DIR
    ${MATLAB_DIR}/bin/glnxa64
)

# Find catkin (the ROS build system)
find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs std_msgs sensor_msgs nav_msgs rosbag)


# Describe catkin Project
catkin_package(
        DEPENDS Eigen3 Boost
        CATKIN_DEPENDS roscpp nav_msgs std_msgs sensor_msgs nav_msgs rosbag
        INCLUDE_DIRS src
)


# Try to compile with c++11
# http://stackoverflow.com/a/25836953
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3  -Wall")



# Include our header files
include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${MATLAB_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND parser_libraries
        ${Boost_LIBRARIES}
        ${Matlab_LIB_DIR}/libmex.so
        ${Matlab_LIB_DIR}/libmx.so
        ${Matlab_LIB_DIR}/libeng.so
        ${Matlab_LIB_DIR}/libmat.so
        ${catkin_LIBRARIES}
)


##################################################
# Make binary for the offline reader
##################################################
add_executable(bagconvert
        src/main.cpp
)
target_link_libraries(bagconvert ${parser_libraries})
```

接下来，从`src`文件夹退出到`kalibr_allan`文件夹，并执行编译命令：

```shell
cd ..
catkin_make -j -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

## 2.数据解算

本次用来测试的数据信息如下：
> xsens-MTI-100：100Hz
> 
> [数据链接](https://pan.baidu.com/s/1i64xkgP)

使用[kalibr_allan](https://github.com/rpng/kalibr_allan)解算，得到如下解算结果：



使用[imu_utils0](https://github.com/gaowenliang/imu_utils)解算，得到如下解算结果：

```yaml
%YAML:1.0
---
type: IMU
name: xsens
Gyr:
   unit: " rad/s"
   avg-axis:
      gyr_n: 1.5763452161529142e-03
      gyr_w: 5.8249564588765805e-05
   x-axis:
      gyr_n: 1.5899761424151017e-03
      gyr_w: 6.1919879182166442e-05
   y-axis:
      gyr_n: 1.5287139519223822e-03
      gyr_w: 6.7266758753347237e-05
   z-axis:
      gyr_n: 1.6103455541212586e-03
      gyr_w: 4.5562055830783737e-05
Acc:
   unit: " m/s^2"
   avg-axis:
      acc_n: 7.7486519052309263e-03
      acc_w: 3.0161169000531306e-04
   x-axis:
      acc_n: 8.0541059960446311e-03
      acc_w: 4.4075230984747259e-04
   y-axis:
      acc_n: 7.6287483999571302e-03
      acc_w: 1.9645119255914294e-04
   z-axis:
      acc_n: 7.5631013196910167e-03
      acc_w: 2.6763156760932364e-04
```

使用[imu_utils1](https://github.com/mintar/imu_utils.git)解算，得到如下解算结果：

```yaml
%YAML:1.0
---
type: IMU
name: xsens
Gyr:
   unit: "gyr_n: rad / sqrt(s), gyr_w: rad / s^2 / sqrt(Hz)"
   avg-axis:
      gyr_n: 1.4707696586409861e-04
      gyr_w: 1.7119540146101773e-06
   x-axis:
      gyr_n: 1.4684344574184067e-04
      gyr_w: 2.7936775470456474e-09
   y-axis:
      gyr_n: 1.3869590134848802e-04
      gyr_w: 2.4433840290090893e-06
   z-axis:
      gyr_n: 1.5569155050196710e-04
      gyr_w: 2.6896843372743968e-06
Acc:
   unit: "acc_n: m / s^2 / sqrt(Hz), acc_w: m / s^3 / sqrt(Hz)"
   avg-axis:
      acc_n: 7.2353358839510656e-04
      acc_w: 3.5380419324294502e-05
   x-axis:
      acc_n: 7.1221224615263077e-04
      acc_w: 7.5011288440083196e-05
   y-axis:
      acc_n: 7.3801506370448154e-04
      acc_w: 2.0665543937216876e-06
   z-axis:
      acc_n: 7.2037345532820717e-04
      acc_w: 2.9063415139078617e-05
```

可以看到，并非是直接乘以频率（Hz）参数的一种变换。
建议使用[imu_utils1](https://github.com/mintar/imu_utils.git)的标定结果，与[kalibr](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model#from-the-allan-standard-deviation-ad)所需要的噪声一致
