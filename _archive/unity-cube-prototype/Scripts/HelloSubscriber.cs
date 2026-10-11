using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

// HelloSubscriber.cs - receives std_msgs/String messages from ROS and logs them.
public class HelloSubscriber : MonoBehaviour
{
    [SerializeField] private string topicName = "ros_hello";

    private ROSConnection ros;

    void Start()
    {
        // Same connection object TestPublisher uses
        ros = ROSConnection.GetOrCreateInstance();

        // Tell the connector: "call OnMessage every time a String arrives on this topic"
        ros.Subscribe<StringMsg>(topicName, OnMessage);
    }

    // This is a "callback": we never call it ourselves, ROS-TCP-Connector calls it for us.
    void OnMessage(StringMsg msg)
    {
        Debug.Log("Unity received: " + msg.data);
    }
}