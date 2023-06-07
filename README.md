# 功能介绍

hobot_visualization package 是地平线基于Ros2开发的 ai_msgs 话题消息转换为 visualization_msgs 话题消息，用于可视化相关数据。


# 编译

## 依赖库

ros package：

- ai_msgs
- rclcpp
- visualization_msgs

'ai_msgs' 是地平线自定义的消息格式，用于算法模型推理后，发布推理结果，ai_msgs package定义在hobot_msgs中。

'rclcpp' 是 ROS2 中的一个C++客户端库，提供了用于创建 ROS2 节点、订阅和发布话题、调用服务、创建定时器等功能的API。

'visualization_msgs' 是ROS2 中的一个消息包，用于可视化和显示机器人相关的数据。

## 开发环境

- 编程语言: C/C++
- 开发平台: X3/X86
- 系统版本：Ubuntu 20.04
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## 编译

 支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

### 编译选项

1. BUILD_HBMEM
   - 零拷贝传输方式使能开关。Docker交叉编译时默认打开(ON), 编译时可以通过-DBUILD_HBMEM=OFF关闭。
   - 在板端编译时，零拷贝传输方式使能开关默认是关闭的。如果需要依赖零拷贝，可以通过-DBUILD_HBMEM=ON打开。
   - 如果打开，编译会依赖hbm_img_msgs package，并且需要使用tros进行编译。
   - 如果关闭，编译和运行不依赖hbm_img_msgs pkg，支持使用原生ros和tros进行编译。
   - 对于零拷贝通信方式，当前只支持订阅nv12格式图片。

### Ubuntu板端编译X3版本

1. 编译环境确认
   - 板端已安装X3 Ubuntu系统。
   - 当前编译终端已设置TogetheROS环境变量：`source PATH/setup.bash`。其中PATH为TogetheROS的安装路径。
   - 已安装ROS2编译工具colcon，安装命令：`pip install -U colcon-common-extensions`

2. 编译
 编译命令：`colcon build --packages-select hobot_visualization --cmake-args -DBUILD_HBMEM=ON`


### Docker交叉编译X3版本

1. 编译环境确认

   - 在docker中编译，并且docker中已经安装好TogetheROS。docker安装、交叉编译说明、TogetheROS编译和部署说明详见机器人开发平台 robot_dev_config repo中的README.md。

2. 编译

   - 编译命令：

```shell
export TARGET_ARCH=aarch64
export TARGET_TRIPLE=aarch64-linux-gnu
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

colcon build --packages-select hobot_visualization \
   --merge-install \
   --cmake-force-configure \
   --cmake-args \
   --no-warn-unused-cli \
   -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
```

### X86 Ubuntu系统上编译 X86版本

1. 编译环境确认

   - x86 ubuntu版本: ubuntu20.04

2. 编译

   - 编译命令：

   ```shell
   colcon build --packages-select hobot_visualization  \
      --merge-install \
      --cmake-args \
      -DPLATFORM_X86=ON \
      -DBUILD_HBMEM=ON \
      -DTHIRD_PARTY=`pwd`/../sysroot_docker \
   ```

## 注意事项

# 使用介绍

## 参数

| 参数名                 | 类型        | 解释                                        | 是否必须 | 支持的配置           | 默认值                        |
| ---------------------- | ----------- | ------------------------------------------- | -------- | -------------------- | ----------------------------- |
| msg_pub_topic_name  | std::string | imagemarker转换为 | 否      | 根据实际部署环境配置 | /hobot_foxglove |
| smart_msg_sub_topic_name  | std::string | 接收dnn节点的的topic名 | 否      | 需要与 ai_msgs 对应的话题名配置一致 | /hobot_agent |

## 运行

编译成功后，将生成的install路径拷贝到地平线旭日X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行：


### **Ubuntu X3**

```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

# 启动visualization node
ros2 run hobot_visualization hobot_visualization

```

### **Ubuntu X3 Launch启动**

```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

# 启动visualization node
ros2 launch hobot_visualization hobot_vis_render.launch.py
```

### **Linux X3**

```shell
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# 启动visualization node检测node
./install/lib/hobot_visualization/hobot_visualization
```

## X86 Ubuntu系统上运行

```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

ros2 launch hobot_visualization hobot_vis_render.launch.py
```


# 结果分析

## X3 日志信息

```bash
[WARN] [1686109522.331820512] [VisNode]: This is hobot visualization render node!
[WARN] [1686109522.401623519] [hobot_trigger]: Parameter:
 msg_pub_topic_name: /hobot_visualization
 smart_msg_sub_topic_name: /hobot_dnn_detection
[INFO] [1686109522.413797638] [VisNode]: VisNode start.
[INFO] [1686109524.536023791] [VisNode]: smart msg: Recved msg, frame_id: 2233, stamp: 1686109524_471390976, targets size: 1 has roi num: 1 has capture num: 0, roi type: surfboard, roi x1: 1, roi y1: 70, roi x2: 264, roi y2: 215, has attr num: 0

[INFO] [1686109524.573302027] [VisNode]: smart msg: Recved msg, frame_id: 2234, stamp: 1686109524_505037056, targets size: 1 has roi num: 1 has capture num: 0, roi type: surfboard, roi x1: 1, roi y1: 68, roi x2: 264, roi y2: 218, has attr num: 0
```

## foxglove 效果展示
foxglove下，visualization msg 和 image融合效果渲染图：
![image](./render/visualization_render.png)