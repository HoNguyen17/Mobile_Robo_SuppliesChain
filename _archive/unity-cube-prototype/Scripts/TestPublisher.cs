using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class TestPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "unity_hello";
    public float publishInterval = 0.5f;
    float timer;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(topicName);
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer > publishInterval)
        {
            ros.Publish(topicName, new StringMsg("hello from Unity " + Time.time));
            timer = 0;
        }
    }
}
