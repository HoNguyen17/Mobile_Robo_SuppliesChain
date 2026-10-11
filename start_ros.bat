@echo off
title Warehouse ROS 1 Backend
echo ========================================================
echo  Dang khoi dong ROS 1 Backend & RViz (WSL Ubuntu-20.04)...
echo  Cong ket noi Unity: 10000
echo ========================================================
wsl -d Ubuntu-20.04 bash -c "killall -9 rosmaster roslaunch python3 rviz gzserver 2>/dev/null; sleep 1; source /opt/ros/noetic/setup.bash && source ~/nav_ws/devel/setup.bash && roslaunch warehouse_bringup bringup.launch"
pause
