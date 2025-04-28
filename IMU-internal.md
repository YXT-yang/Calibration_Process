# IMU-internal parameters

因为在标定IMU内参的过程中遇到了数据单位的问题，在查看[imu_utils0](https://github.com/gaowenliang/imu_utils)的issue的过程中，我发现有人同样提出了类似的问题，并给出了自己所撰写的代码[imu_utils1](https://github.com/mintar/imu_utils.git)，现在我通过解算同一组数据，验证一下[imu_utils0](https://github.com/gaowenliang/imu_utils)解算得到的参数的单位。

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

## 2.数据解算

本次用来测试的数据信息如下：
> xsens-MTI-100：100Hz
> [数据链接](https://pan.baidu.com/s/1i64xkgP)
