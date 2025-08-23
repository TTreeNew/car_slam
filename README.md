# ros2 + NAvigation2 自动巡检，拍照，语音播放
## 1.项目介绍
按照fishros书籍第七章制作。git使用练习。项目开发环境：
- ubuntu22.04
- ros版本： ros2 humble
- 使用ros2_control实现运动控制，建图使用slam-toolbox,导航使用Navigation2，仿真使用Gazebo
- 库由包管理器apt和pip3安装
## 2.使用方法
```bash
colcon build
source install/setup.bash
```
### 2.1 依赖
依赖在各个package.xml的`<depend>`里面
### 2.2 运行
首先在car_slam_test下打开终端，构建功能包
```bash
colcon build
```
运行gazebo仿真：在car_slam_test下打开新的终端，运行
```bash
source install/setup.bash
ros2 launch car_description_pkg gazebo_sim.launch.py
```
运行rviz2可视化导航：在car_slam_test下打开新的终端，运行
```bash
source install/setup.bash
ros2 ros2 launch carbot_navigation2 navigation2.launch.py 
```
运行自动巡检：在car_slam_test下打开新的终端，运行
```bash
source install/setup.bash
ros2 launch autopatrol_robot autopatrol.launch.py 
```

## 3.功能包说明
- autopatrol_robot：自动巡检功能包，包含自动巡检、拍照、语音播放完整功能实现的功能包
- autopatrol_interfaces：自定义接口功能包。当前只自定义了语音播放的服务接口
- car_description_pkg：包含仿真urdf文件，包括机器人建模，环境建模，ros2_control配置。
- carbot_application：包含机器人小功能的实现。init_robot_pose：初始化当前位姿，运行后在地图起始位置出现机器人。
get_robot_pose：监听tf变换，获取机器人当前坐标并打印在日志上。nav_to_pose：导航向程序里设定好的目标点
waypoint_follower：沿着程序里设定好的多个点导航。
- carbot_navigation2：实现导航可视化，有nav2配置文件和slam-toolbox保存的地图文件。启动launch后会加载包含rviz2界面布局的配置文件和启动navigation2

## 4.作者
- [fishros](https://github.com/fishros/ros2bookcode/blob/master/chapt7)