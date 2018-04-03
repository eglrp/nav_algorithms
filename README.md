# nav_algorithms
Some mobile robots's navigation algorithms.

Q-learning:[参考博客](https://juejin.im/entry/5a435462f265da43333eaebe)  
 [专栏](https://jizhi.im/blog/post/intro_q_learning)
先大致写些内容,会比较乱.
两大部分:定位和路径规划.
- 路径规划:move_base包主要包括global and local by costmap.
           kill %
     	costmap_2d介绍
 	write a global planner as plugin in ROS
- 定位:主动蒙特卡罗粒子滤波定位算法 AMCL

map_server:提供了为ROS服务的地图数据.
URDF:Unified Robot Description Format，统一机器人描述格式.XACRO格式提供了一些更高级的方式来组织编辑机器人描述.[参考](https://blog.csdn.net/sunbibei/article/details/52297524)

