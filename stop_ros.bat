@echo off
title Clean Stop ROS
echo Dang dung toan bo tien trinh ROS tren WSL...
wsl -d Ubuntu-20.04 bash -c "killall -9 rosmaster roslaunch python3 rviz gzserver 2>/dev/null"
echo Da dung xong toan bo tien trinh ROS!
timeout /t 2 >nul
