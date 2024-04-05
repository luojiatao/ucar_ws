# 使用指南

## 启动Gazebo仿真环境、AMCL自定位和Move_Base自主导航

运行以下脚本：

```bash
sh start.sh

图像二值化处理
binary_image.py 文件负责订阅ROS图像主题，将接收到的图像进行二值化处理。

直线检测
line_detection.py 文件接收二值化后的图像，并应用直线检测算法。