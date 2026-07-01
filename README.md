# 1.关于莫师兄给的仿真环境的使用说明
## 1.1 环境介绍
### 为了保证仿真环境能够正常加载出来，对github中的源代码进行了简化，以及将ros1环境替换成ros2环境。
### 该环境的运行脚本在agribot_gazebo/launch/agribot_farm.launch.py 中
在运行这个脚本时，终端会报错，但是不影响加载和运行
## 1.2 关于导航算法
### 该环境中使用的导航算法为dwa导航算法(询问AI的结果)，该算法在agribot_visual_servoing/launch/visual_visual_servoing.launch.py 中
主要的参数为：
- linear_velocity: 0.5  # 线速度
- angular_velocity: 0.5  # 角速度
- linear_acceleration: 0.2  # 线加速度
- angular_acceleration: 0.5  # 角加速度
- v_max: 0.3  # 最大线速度
- w_max: 0.0  # 最大角速度

HSV颜色阈值： 
- 'green_h_low': 25  # 色调下限
- 'green_s_low': 30  # 饱和度下限（去掉灰白背景）
- 'green_v_low': 30  # 亮度下限（去掉阴影）
- 'green_h_high': 95 # 色调上限
- 'green_s_high': 255 # 饱和度上限
- 'green_v_high': 255 # 亮度上限
这个是固定识别绿色，如果想改变作物，可能需要重新调参

## 1.3目前的问题
不知道为什么在rviz2中的前置相机画面中，能将作物连成一条直线，但是没有中间线的显示，我记得之前是有的，在将拟合线稳定后(虽然现在也不怎么稳定，但至少比之前稳定)，中间线就消失了(哭了)。
还有就是转弯问题依然存在

就是考试周，时间紧迫，所以调试的时间很少(憨笑)，如果需要的话，7.9考完试就可以去实验室继续调试，但是我感觉有点费时间
