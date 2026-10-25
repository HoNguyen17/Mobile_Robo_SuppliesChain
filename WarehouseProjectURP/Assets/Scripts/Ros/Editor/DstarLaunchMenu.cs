using UnityEditor;
using UnityEngine;

/// <summary>
/// Menu item for launching the clean ROS system with D* Lite planner in WSL.
/// Conservative C# for Unity 2021.1 (.NET Standard 2.0).
/// </summary>
public static class DstarLaunchMenu
{
    [MenuItem("Warehouse/1D. Launch ROS System with D* Lite (WSL)", false, 2)]
    [MenuItem("Robotics/Warehouse/1D. Launch ROS System with D* Lite (WSL)", false, 1001)]
    public static void LaunchRosSystemDstar()
    {
        System.Diagnostics.ProcessStartInfo psi = new System.Diagnostics.ProcessStartInfo
        {
            FileName = "wsl.exe",
            Arguments = "-d Ubuntu-20.04 bash -c \"source /opt/ros/noetic/setup.bash && source ~/nav_ws/devel/setup.bash && roslaunch warehouse_bringup bringup_dstar.launch\"",
            UseShellExecute = true
        };
        System.Diagnostics.Process.Start(psi);
        UnityEngine.Debug.Log("[ROS LAUNCH] Starting clean ROS System with D* Lite & RViz in WSL...");
    }
}
