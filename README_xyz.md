# README


---

1. aruco_ros 是我增加的文件夹
    aruco_ros/aruco_ros.hpp aruco_ros.cpp 中有类 Listener实现：

    * 监听 optitrack 和 aruco node 发送的 pose msg
    * 时间同步，计算误差
    * 发送参考姿态，位置姿态误差
    * 进行安装标定
    
aruco_ros/EigenLM.hpp EigenLM.cpp 中包含供安装标定使用的 LM 相关函数。
    
2. aruco 原本函数只修改了一个文件： utils_markermap/aruco_test_markermap.cpp
    在其中创建了一个节点发送aruco的pose msg

3. AllRelease 中包含所有可执行文件
- pub_calibInstall.sh 发送标定命令，并可选择标定方法 和 在线采集数据/读取标定数据包
- runOpti_node.sh 发送启动optiTrack节点，设置刚体ID。该节点存储在mocap_node_devel中
- pub_arucoDealy.sh 发送延迟調整命令






