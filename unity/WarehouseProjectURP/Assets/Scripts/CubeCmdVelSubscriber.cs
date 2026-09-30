using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

// CubeCmdVelSubscriber.cs - drives this object from /cmd_vel (geometry_msgs/Twist).
// linear.x  = forward speed in m/s
// angular.z = turn rate in rad/s (ROS convention: positive = turn left)
public class CubeCmdVelSubscriber : MonoBehaviour
{
    [SerializeField] private string topicName = "/cmd_vel";
    [SerializeField] private float commandTimeout = 0.5f;   // stop if no command for this long

    private float linearX;
    private float angularZ;
    private float lastCommandTime = -999f;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<TwistMsg>(topicName, OnCmdVel);
    }

    // Called by ROS-TCP-Connector whenever a Twist arrives. We only store the values.
    void OnCmdVel(TwistMsg msg)
    {
        linearX = (float)msg.linear.x;
        angularZ = (float)msg.angular.z;
        lastCommandTime = Time.time;
    }

    void Update()
    {
        // Dead-man switch: no fresh command -> stop
        if (Time.time - lastCommandTime > commandTimeout)
        {
            linearX = 0f;
            angularZ = 0f;
        }

        // Unity's Y rotation is clockwise-positive, ROS yaw is counter-clockwise-positive,
        // hence the minus sign.
        transform.Rotate(0f, -angularZ * Mathf.Rad2Deg * Time.deltaTime, 0f);

        // 1 Unity unit = 1 metre
        transform.position += transform.forward * (linearX * Time.deltaTime);
    }
}