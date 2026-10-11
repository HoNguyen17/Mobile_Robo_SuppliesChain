using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Publishes the robot's odometry on /odom and the odom -> base_footprint transform on /tf.
///
/// The odom frame starts where the robot is when Play starts (like real wheel odometry). The pose is read from
/// the simulated body, so this odometry does not drift. Navigation does not use it: the map -> base_footprint
/// transform comes from RobotPosePublisher (ground truth, ADR-015), so publishTf is normally off.
/// Positions and speeds are divided by the robot scale, so ROS sees true robot metres (ADR-010).
///
/// Put this on the robot's ROOT GameObject, next to DiffDriveController.
/// </summary>
[RequireComponent(typeof(DiffDriveController))]
[DisallowMultipleComponent]
public class OdometryPublisher : MonoBehaviour
{
    public string odomTopic = "/odom";
    public string tfTopic = "/tf";
    public string odomFrame = "odom";
    public string baseFrame = "base_footprint";
    public float rateHz = 30f;

    [Tooltip("Also publish odom -> base_footprint on /tf. Untick only if another node publishes it.")]
    public bool publishTf = true;

    // Small, fixed confidence for the planar axes; huge for the unused ones (z, roll, pitch).
    private const double Known = 1e-3;
    private const double Unused = 1e6;

    private ROSConnection ros;
    private DiffDriveController drive;
    private ArticulationBody body;
    private PublishTimer timer;
    private Vector3 startPosition;
    private float startYawDeg;
    private uint seq;

    void Start()
    {
        drive = GetComponent<DiffDriveController>();
        body = drive.BaseBody.GetComponent<ArticulationBody>();
        startPosition = drive.BaseBody.position;
        startYawDeg = RosConversions.UnityYawDegrees(drive.BaseBody.forward);
        timer = new PublishTimer(rateHz);

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OdometryMsg>(odomTopic);
        if (publishTf) ros.RegisterPublisher<TFMessageMsg>(tfTopic);
    }

    void Update()
    {
        double now = Time.timeAsDouble;
        if (!timer.Tick(now)) return;

        Transform baseBody = drive.BaseBody;
        float scale = transform.lossyScale.x;
        float yawDeg = RosConversions.UnityYawDegrees(baseBody.forward);
        Pose2D pose = RosConversions.ToOdomPose(startPosition, startYawDeg, baseBody.position, yawDeg, scale);

        double linear = 0.0;
        double angular = 0.0;
        if (body != null)
        {
            linear = RosConversions.ForwardSpeed(body.velocity, baseBody.forward, scale);
            angular = RosConversions.YawRate(body.angularVelocity);
        }

        HeaderMsg header = new HeaderMsg(seq++, RosConversions.ToRosTime(now), odomFrame);
        QuaternionMsg orientation = RosConversions.YawToQuaternion(pose.Yaw);

        ros.Publish(odomTopic, BuildOdometry(header, pose, orientation, linear, angular));
        if (publishTf) ros.Publish(tfTopic, BuildTf(header, pose, orientation));
    }

    private OdometryMsg BuildOdometry(HeaderMsg header, Pose2D pose, QuaternionMsg orientation, double linear, double angular)
    {
        PoseMsg poseMsg = new PoseMsg(new PointMsg(pose.X, pose.Y, 0.0), orientation);
        TwistMsg twist = new TwistMsg(new Vector3Msg(linear, 0.0, 0.0), new Vector3Msg(0.0, 0.0, angular));
        return new OdometryMsg(
            header,
            baseFrame,
            new PoseWithCovarianceMsg(poseMsg, PlanarCovariance()),
            new TwistWithCovarianceMsg(twist, PlanarCovariance()));
    }

    private TFMessageMsg BuildTf(HeaderMsg header, Pose2D pose, QuaternionMsg orientation)
    {
        TransformMsg tf = new TransformMsg(new Vector3Msg(pose.X, pose.Y, 0.0), orientation);
        return new TFMessageMsg(new[] { new TransformStampedMsg(header, baseFrame, tf) });
    }

    /// <summary>6x6 row-major covariance over (x, y, z, roll, pitch, yaw).</summary>
    private static double[] PlanarCovariance()
    {
        double[] c = new double[36];
        c[0] = Known;   // x
        c[7] = Known;   // y
        c[14] = Unused; // z
        c[21] = Unused; // roll
        c[28] = Unused; // pitch
        c[35] = Known;  // yaw
        return c;
    }
}
