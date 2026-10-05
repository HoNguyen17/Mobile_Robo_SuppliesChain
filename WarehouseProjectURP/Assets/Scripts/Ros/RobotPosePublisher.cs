using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Publishes the robot's ground-truth pose in the ROS "map" frame (ADR-012): /robot/pose (PoseStamped) and the
/// transform map -> base_footprint on /tf. The pose is read from the simulated body and is not estimated, so
/// it never drifts. Positions are divided by the robot scale, so ROS sees true robot metres (ADR-010), and the
/// stamp is simulation time (ADR-009).
///
/// /odom (OdometryPublisher) is still published for RViz, but its TF is switched off: base_footprint has one parent.
///
/// Put this on the robot's ROOT GameObject, next to DiffDriveController.
/// </summary>
[RequireComponent(typeof(DiffDriveController))]
[DisallowMultipleComponent]
public class RobotPosePublisher : MonoBehaviour
{
    public string poseTopic = "/robot/pose";
    public string tfTopic = "/tf";
    public string mapFrame = "map";
    public string baseFrame = "base_footprint";
    public float rateHz = 30f;

    [Tooltip("Also publish map -> base_footprint on /tf (RViz needs it to draw the robot model in the map frame).")]
    public bool publishTf = true;

    [Tooltip("At Start, write the map-frame position of the navigator's target shelf and home point to the Console, " +
             "so they can be used as goals for the ROS planner.")]
    public bool logNavigatorPoints = true;

    private ROSConnection ros;
    private DiffDriveController drive;
    private PublishTimer timer;
    private uint seq;

    void Start()
    {
        drive = GetComponent<DiffDriveController>();
        timer = new PublishTimer(rateHz);

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(poseTopic);
        if (publishTf) ros.RegisterPublisher<TFMessageMsg>(tfTopic);

        if (logNavigatorPoints) LogNavigatorPoints();
    }

    void Update()
    {
        double now = Time.timeAsDouble;
        if (!timer.Tick(now)) return;

        Transform body = drive.BaseBody;
        float yawDeg = RosConversions.UnityYawDegrees(body.forward);
        Pose2D pose = RosConversions.ToMapPose(body.position, yawDeg, transform.lossyScale.x);

        HeaderMsg header = new HeaderMsg(seq++, RosConversions.ToRosTime(now), mapFrame);
        QuaternionMsg orientation = RosConversions.YawToQuaternion(pose.Yaw);

        PoseMsg poseMsg = new PoseMsg(new PointMsg(pose.X, pose.Y, 0.0), orientation);
        ros.Publish(poseTopic, new PoseStampedMsg(header, poseMsg));

        if (publishTf)
        {
            TransformMsg tf = new TransformMsg(new Vector3Msg(pose.X, pose.Y, 0.0), orientation);
            ros.Publish(tfTopic, new TFMessageMsg(new[] { new TransformStampedMsg(header, baseFrame, tf) }));
        }
    }

    private void LogNavigatorPoints()
    {
        TurtleBotNavigator navigator = GetComponent<TurtleBotNavigator>();
        if (navigator == null) return;

        float scale = transform.lossyScale.x;
        string text = "RobotPosePublisher: positions in the ROS map frame (robot metres):";
        if (navigator.homePoint != null) text += Line("home point", navigator.homePoint.position, scale);
        if (navigator.target != null) text += Line("target shelf", navigator.target.position, scale);
        text += Line("robot now", drive.BaseBody.position, scale);
        Debug.Log(text, this);
    }

    private static string Line(string label, Vector3 unityPosition, float scale)
    {
        Pose2D p = RosConversions.ToMapPose(unityPosition, 0f, scale);
        return "\n  " + label + ": x=" + p.X.ToString("F2") + " y=" + p.Y.ToString("F2");
    }
}
