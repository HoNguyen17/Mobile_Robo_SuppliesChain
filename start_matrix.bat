@echo off
title Warehouse Benchmark Matrix
echo ========================================================
echo  Unattended benchmark matrix (WSL Ubuntu-20.04)
echo.
echo  1. Unity must be in PLAY mode (wait 90 s after pressing Play).
echo  2. The laptop must be plugged in and set to never sleep.
echo  3. Do not touch Unity until it finishes. Do not use
echo     start_ros.bat, stop_ros.bat or Unity menu 3: they kill
echo     every python3 process, including this matrix.
echo.
echo  Default: S-00c S-04 S-06 S-07 S-08 x astar dstar x N=20.
echo  Examples:  start_matrix.bat --dry
echo             start_matrix.bat --preflight-only
echo             start_matrix.bat --scenarios S-04,S-06 --repeats 10
echo             start_matrix.bat --resume ~/ab_matrix/^<folder^>
echo  Stop with Ctrl-C (it ends the current attempt and writes the report).
echo ========================================================
wsl -d Ubuntu-20.04 --cd "%~dp0." -e bash -c "source /opt/ros/noetic/setup.bash && source ~/nav_ws/devel/setup.bash && python3 -u ros1/turtlebot_control/scripts/ab_matrix.py %*"
echo.
echo Report: the last line above names matrix_report.md. From WSL: cat ~/ab_matrix/latest_matrix.txt
pause
