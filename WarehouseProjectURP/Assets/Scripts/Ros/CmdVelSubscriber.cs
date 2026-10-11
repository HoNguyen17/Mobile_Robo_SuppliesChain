using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Drives the robot from ROS /cmd_vel (geometry_msgs/Twist): linear.x in robot m/s, angular.z in rad/s
/// counter-clockwise. DiffDriveController already uses these ROS conventions, so the values pass straight
/// through. If messages stop, DiffDriveController's 0.5 s watchdog stops the wheels.
///
/// The first /cmd_vel switches off the Unity-only TurtleBotNavigator, so the two never fight over the wheels.
///
/// Put this on the robot's ROOT GameObject, next to DiffDriveController.
/// </summary>
[RequireComponent(typeof(DiffDriveController))]
[DisallowMultipleComponent]
public class CmdVelSubscriber : MonoBehaviour
{
    public string topic = "/cmd_vel";

    [Tooltip("Disable TurtleBotNavigator (Unity-only test mode) as soon as ROS sends a command.")]
    public bool takeOverFromUnityNavigator = true;

    private ROSConnection ros;
    private DiffDriveController drive;
    private TurtleBotNavigator navigator;

    void Start()
    {
        drive = GetComponent<DiffDriveController>();
        navigator = GetComponent<TurtleBotNavigator>();
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(topic, OnCmdVel);
    }

    void OnDestroy()
    {
        if (ros != null) ros.Unsubscribe(topic);
    }

    private void OnCmdVel(TwistMsg msg)
    {
        if (this == null || !enabled) return;

        if (takeOverFromUnityNavigator && navigator != null && navigator.enabled)
        {
            navigator.enabled = false;
            Debug.Log("CmdVelSubscriber: ROS is driving now; TurtleBotNavigator disabled.", this);
        }
        drive.SetCommand((float)msg.linear.x, (float)msg.angular.z);
    }
}
