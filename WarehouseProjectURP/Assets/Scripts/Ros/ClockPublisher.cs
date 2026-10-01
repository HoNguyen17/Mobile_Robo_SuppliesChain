using RosMessageTypes.Rosgraph;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Publishes Unity's simulation time on /clock (ADR-009). ROS runs with use_sim_time, so when Unity pauses or
/// slows down, ROS time does too.
///
/// Sends once per rendered frame: simulation time only advances per frame, so that is the highest useful rate.
/// It runs before every other script, so in each frame /clock goes out before any message stamped with it.
///
/// Put ONE in the scene (the "Robotics > Warehouse > Add ROS Bridge" menu adds it).
/// </summary>
[DefaultExecutionOrder(-1000)]
[DisallowMultipleComponent]
public class ClockPublisher : MonoBehaviour
{
    public string topic = "/clock";

    private ROSConnection ros;
    private double lastSent = double.NegativeInfinity;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ClockMsg>(topic);
    }

    void Update()
    {
        double now = Time.timeAsDouble;
        if (now <= lastSent) return; // never send the same or an older time twice
        lastSent = now;
        ros.Publish(topic, new ClockMsg(RosConversions.ToRosTime(now)));
    }
}
