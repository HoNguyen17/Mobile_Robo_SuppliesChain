using System;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using UnityEngine;

/// <summary>
/// Pure conversions between Unity and ROS. No MonoBehaviour, no ROS connection, so they are unit-tested.
///
///   Unity: left-handed, X right, Y up, Z forward; yaw is clockwise seen from above.
///   ROS (REP-103): X forward, Y left, Z up; yaw is counter-clockwise.
///
/// The robot is scaled up in Unity (ADR-010). Every length that goes to ROS is divided by the robot scale,
/// so ROS always sees true robot metres.
/// </summary>
public static class RosConversions
{
    private const double NanosPerSecond = 1e9;

    /// <summary>Simulation seconds to a ROS 1 time stamp. Negative times clamp to zero.</summary>
    public static TimeMsg ToRosTime(double seconds)
    {
        if (seconds <= 0.0) return new TimeMsg(0u, 0u);
        double whole = Math.Floor(seconds);
        long nanos = (long)Math.Round((seconds - whole) * NanosPerSecond);
        if (nanos >= (long)NanosPerSecond)
        {
            whole += 1.0;
            nanos -= (long)NanosPerSecond;
        }
        return new TimeMsg((uint)whole, (uint)nanos);
    }

    /// <summary>Heading of a Unity direction in degrees, clockwise from +Z. Pitch and roll are ignored.</summary>
    public static float UnityYawDegrees(Vector3 forward)
    {
        return Mathf.Atan2(forward.x, forward.z) * Mathf.Rad2Deg;
    }

    /// <summary>Wraps an angle into (-pi, pi].</summary>
    public static double NormalizeAngle(double radians)
    {
        double a = Math.IEEERemainder(radians, 2.0 * Math.PI);
        return a <= -Math.PI ? a + 2.0 * Math.PI : a;
    }

    /// <summary>A rotation about the ROS Z axis.</summary>
    public static QuaternionMsg YawToQuaternion(double yaw)
    {
        return new QuaternionMsg(0.0, 0.0, Math.Sin(yaw * 0.5), Math.Cos(yaw * 0.5));
    }

    /// <summary>
    /// The robot's pose in the odom frame, whose origin is the robot's pose at start (like a real wheel-odometry
    /// frame). Height is ignored: the navigation stack is planar.
    /// </summary>
    public static Pose2D ToOdomPose(Vector3 startPosition, float startYawDeg, Vector3 position, float yawDeg, float robotScale)
    {
        Vector3 delta = position - startPosition;
        delta.y = 0f;
        Vector3 local = Quaternion.Euler(0f, -startYawDeg, 0f) * delta; // into the start frame (Unity axes)

        double x = local.z / robotScale;   // Unity forward -> ROS +X
        double y = -local.x / robotScale;  // Unity right   -> ROS -Y
        double yaw = NormalizeAngle(-(yawDeg - startYawDeg) * Mathf.Deg2Rad);
        return new Pose2D(x, y, yaw);
    }

    /// <summary>
    /// A pose in the ROS "map" frame (ADR-012): Unity's world turned into ROS axes and divided by the robot scale,
    /// so the frame's origin is the Unity world origin. Same as the odom pose of a robot that started at the origin.
    /// </summary>
    public static Pose2D ToMapPose(Vector3 position, float yawDeg, float robotScale)
    {
        return ToOdomPose(Vector3.zero, 0f, position, yawDeg, robotScale);
    }

    /// <summary>Forward speed in robot m/s: the world velocity projected on the heading, divided by the scale.</summary>
    public static double ForwardSpeed(Vector3 velocity, Vector3 forward, float robotScale)
    {
        forward.y = 0f;
        if (forward.sqrMagnitude < 1e-12f) return 0.0;
        return Vector3.Dot(velocity, forward.normalized) / robotScale;
    }

    /// <summary>Yaw rate in rad/s, counter-clockwise positive. Angular speed does not depend on scale.</summary>
    public static double YawRate(Vector3 angularVelocity)
    {
        return -angularVelocity.y;
    }

    /// <summary>
    /// World direction of a horizontal LIDAR beam. <paramref name="angle"/> is the ROS beam angle
    /// (0 = robot forward, positive = to the left).
    /// </summary>
    public static Vector3 ScanDirection(float yawDeg, double angle)
    {
        // A left (counter-clockwise) beam angle is a negative Unity yaw.
        float worldYaw = (yawDeg - (float)(angle * Mathf.Rad2Deg)) * Mathf.Deg2Rad;
        return new Vector3(Mathf.Sin(worldYaw), 0f, Mathf.Cos(worldYaw));
    }
}
