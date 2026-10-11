@echo off
title Warehouse Robot Mission
echo ========================================================
echo  Dang chay Nhiem vu Robot Kho (WSL Ubuntu-20.04)...
echo  Kich ban: S-00
echo ========================================================
wsl -d Ubuntu-20.04 bash -c "source /opt/ros/noetic/setup.bash && source ~/nav_ws/devel/setup.bash && python3 ~/nav_ws/src/warehouse_mission/scripts/mission_node.py --scenario S-00"
pause
