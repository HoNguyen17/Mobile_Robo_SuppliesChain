using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;

// CubePosePublisher.cs - publishes this object's position and orientation
// to ROS as geometry_msgs/Pose, converted to the ROS coordinate convention.
public class CubePosePublisher : MonoBehaviour
{
    [SerializeField] private string topicName = "/cube/pose";
    [SerializeField] private float publishRateHz = 10f;

    private ROSConnection ros;
    private float timer;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseMsg>(topicName);
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer < 1f / publishRateHz) return;
        timer = 0f;

        PoseMsg msg = new PoseMsg
        {
            position = transform.position.To<FLU>(),
            orientation = transform.rotation.To<FLU>()
        };
        ros.Publish(topicName, msg);
    }
}