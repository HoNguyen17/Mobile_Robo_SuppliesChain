using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Publishes a 360-beam raycast LIDAR on /scan, modelled on the TurtleBot3 LDS-01: 0.12-3.5 m, 5 Hz,
/// 1 degree per beam, counter-clockwise from robot forward. Ranges are in robot metres (ADR-010), and
/// "no return" is +Infinity (REP-117).
///
/// The beams start at the 'base_scan' link of the imported URDF. Its offset from base_footprint reaches
/// ROS through robot_state_publisher, so this script only sends the ranges.
///
/// Put this on the robot's ROOT GameObject.
/// </summary>
[DisallowMultipleComponent]
public class LaserScanPublisher : MonoBehaviour
{
    public string topic = "/scan";
    public string frameId = "base_scan";

    [Tooltip("Name of the URDF link the beams start from.")]
    public string scanLinkName = "base_scan";

    [Header("LDS-01 (robot metres)")]
    public int beams = 360;
    public float rangeMin = 0.12f;
    public float rangeMax = 3.5f;
    public float rateHz = 5f;

    [Tooltip("Layers the beams can hit.")]
    public LayerMask hitLayers = Physics.DefaultRaycastLayers;

    private ROSConnection ros;
    private LaserScanner scanner;
    private PublishTimer timer;
    private Transform scanLink;
    private uint seq;

    void Start()
    {
        scanLink = FindDeep(transform, scanLinkName);
        if (scanLink == null)
        {
            Debug.LogError("LaserScanPublisher: no child named '" + scanLinkName + "' under '" + name + "'.", this);
            enabled = false;
            return;
        }

        scanner = new LaserScanner(beams, rangeMin, rangeMax, transform, hitLayers);
        timer = new PublishTimer(rateHz);
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(topic);
    }

    void Update()
    {
        double now = Time.timeAsDouble;
        if (!timer.Tick(now)) return;

        float yawDeg = RosConversions.UnityYawDegrees(scanLink.forward);
        float[] ranges = scanner.Scan(scanLink.position, yawDeg, transform.lossyScale.x);
        float increment = scanner.AngleIncrement;

        LaserScanMsg msg = new LaserScanMsg(
            new HeaderMsg(seq++, RosConversions.ToRosTime(now), frameId),
            0f,                               // angle_min
            increment * (scanner.BeamCount - 1), // angle_max
            increment,
            0f,                               // time_increment: all beams are cast at once
            1f / rateHz,                      // scan_time
            scanner.RangeMinM,
            scanner.RangeMaxM,
            ranges,
            new float[0]);                    // no intensities
        ros.Publish(topic, msg);
    }

    private static Transform FindDeep(Transform root, string childName)
    {
        foreach (Transform t in root.GetComponentsInChildren<Transform>(true))
        {
            if (t.name == childName) return t;
        }
        return null;
    }
}
