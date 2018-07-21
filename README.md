# nav_algorithms
Some mobile robots's navigation algorithms.

Q-learning:[参考博客](https://juejin.im/entry/5a435462f265da43333eaebe)  
 [专栏](https://jizhi.im/blog/post/intro_q_learning)
先大致写些内容,会比较乱.
两大部分:定位和路径规划.
- 路径规划:[move_base](http://wiki.ros.org/move_base?distro=lunar)包主要包括global and local by costmap.
          Ctrl + Z挂起后-->jobs查看job号-->kill %num关掉进程(或ps查看PID-->kill PID)
     	costmap_2d介绍
 	write a global planner as plugin in ROS
[此外,还有 nav2d](http://wiki.ros.org/nav2d)
[Dijkstra算法和A*算法的比较](https://blog.csdn.net/wangjingqi930330/article/details/72457584)
	Dijkstra算法的实质是广度优先搜索，是一种发散式的搜索，所以空间复杂度和时间复杂度都比较高。
A*算法不但记录其到源点的代价，还计算当前点到目标点的期望代价，是一种启发式算法，也可以认为是一种深度优先的算法。
- 定位:主动蒙特卡罗粒子滤波定位算法 AMCL

map_server:提供了为ROS服务的地图数据.
URDF:Unified Robot Description Format，统一机器人描述格式.XACRO格式提供了一些更高级的方式来组织编辑机器人描述.[参考](https://blog.csdn.net/sunbibei/article/details/52297524)

### example
>roslaunch exercise_1 exercise_1.launch 为演示路径规划(以A*实现)
>roslaunch exercise_2 exercise_2.launch 为演示轨迹规划
