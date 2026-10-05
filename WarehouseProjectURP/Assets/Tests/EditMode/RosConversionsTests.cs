using System;
using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Unity (left-handed, Y up, Z forward, yaw clockwise) to ROS (REP-103: X forward, Y left, Z up,
/// yaw counter-clockwise), with the robot scaled 4x in Unity (ADR-010).
/// </summary>
public class RosConversionsTests
{
    private const float Scale = 4f;
    private const double Tol = 1e-4;

    // ---- time ---------------------------------------------------------

    [Test]
    public void ToRosTime_splits_seconds_and_nanoseconds()
    {
        var t = RosConversions.ToRosTime(1.5);
        Assert.AreEqual(1u, t.sec);
        Assert.AreEqual(500000000u, t.nanosec);
    }

    [Test]
    public void ToRosTime_carries_rounding_into_seconds()
    {
        var t = RosConversions.ToRosTime(2.9999999999);
        Assert.AreEqual(3u, t.sec);
        Assert.AreEqual(0u, t.nanosec);
    }

    [Test]
    public void ToRosTime_clamps_negative_to_zero()
    {
        var t = RosConversions.ToRosTime(-1.0);
        Assert.AreEqual(0u, t.sec);
        Assert.AreEqual(0u, t.nanosec);
    }

    // ---- yaw ----------------------------------------------------------

    [Test]
    public void UnityYawDegrees_is_clockwise_from_plus_z()
    {
        Assert.AreEqual(0f, RosConversions.UnityYawDegrees(Vector3.forward), 1e-3f);
        Assert.AreEqual(90f, RosConversions.UnityYawDegrees(Vector3.right), 1e-3f);
        Assert.AreEqual(-90f, RosConversions.UnityYawDegrees(Vector3.left), 1e-3f);
    }

    [Test]
    public void UnityYawDegrees_ignores_pitch()
    {
        Vector3 tilted = (Vector3.forward + Vector3.up * 0.3f).normalized;
        Assert.AreEqual(0f, RosConversions.UnityYawDegrees(tilted), 1e-3f);
    }

    [Test]
    public void NormalizeAngle_wraps_into_minus_pi_to_pi()
    {
        Assert.AreEqual(-Math.PI / 2, RosConversions.NormalizeAngle(3 * Math.PI / 2), Tol);
        Assert.AreEqual(Math.PI / 2, RosConversions.NormalizeAngle(-3 * Math.PI / 2), Tol);
        Assert.AreEqual(0.0, RosConversions.NormalizeAngle(2 * Math.PI), Tol);
    }

    [Test]
    public void YawToQuaternion_rotates_about_z()
    {
        var q = RosConversions.YawToQuaternion(Math.PI / 2);
        Assert.AreEqual(0.0, q.x, Tol);
        Assert.AreEqual(0.0, q.y, Tol);
        Assert.AreEqual(Math.Sqrt(0.5), q.z, Tol);
        Assert.AreEqual(Math.Sqrt(0.5), q.w, Tol);
    }

    // ---- odometry pose --------------------------------------------------

    [Test]
    public void OdomPose_forward_in_unity_is_plus_x_in_ros_and_divided_by_scale()
    {
        Pose2D p = RosConversions.ToOdomPose(Vector3.zero, 0f, new Vector3(0f, 0f, 8f), 0f, Scale);
        Assert.AreEqual(2.0, p.X, Tol);
        Assert.AreEqual(0.0, p.Y, Tol);
        Assert.AreEqual(0.0, p.Yaw, Tol);
    }

    [Test]
    public void OdomPose_right_in_unity_is_minus_y_in_ros()
    {
        Pose2D p = RosConversions.ToOdomPose(Vector3.zero, 0f, new Vector3(4f, 0f, 0f), 0f, Scale);
        Assert.AreEqual(0.0, p.X, Tol);
        Assert.AreEqual(-1.0, p.Y, Tol);
    }

    [Test]
    public void OdomPose_clockwise_unity_turn_is_negative_ros_yaw()
    {
        Pose2D p = RosConversions.ToOdomPose(Vector3.zero, 0f, Vector3.zero, 90f, Scale);
        Assert.AreEqual(-Math.PI / 2, p.Yaw, Tol);
    }

    [Test]
    public void OdomPose_is_relative_to_the_start_pose()
    {
        // Robot starts at (10, 0, 10) facing Unity +X, then drives 4 units straight ahead.
        Pose2D p = RosConversions.ToOdomPose(new Vector3(10f, 0f, 10f), 90f, new Vector3(14f, 0f, 10f), 90f, Scale);
        Assert.AreEqual(1.0, p.X, Tol);
        Assert.AreEqual(0.0, p.Y, Tol);
        Assert.AreEqual(0.0, p.Yaw, Tol);
    }

    [Test]
    public void OdomPose_ignores_height()
    {
        Pose2D p = RosConversions.ToOdomPose(Vector3.zero, 0f, new Vector3(0f, 3f, 0f), 0f, Scale);
        Assert.AreEqual(0.0, p.X, Tol);
        Assert.AreEqual(0.0, p.Y, Tol);
    }

    [Test]
    public void OdomPose_yaw_wraps_across_180_degrees()
    {
        // From 170 to -170 (= 190) is a 20 degree clockwise turn.
        Pose2D p = RosConversions.ToOdomPose(Vector3.zero, 170f, Vector3.zero, -170f, Scale);
        Assert.AreEqual(-20.0 * Math.PI / 180.0, p.Yaw, Tol);
    }

    // ---- map pose (ground truth, frame "map" = Unity world origin) -------------

    [Test]
    public void MapPose_uses_ros_axes_robot_metres_and_the_unity_world_origin()
    {
        // Unity z = 8, x = -4 at scale 4 is ROS x = 2 m, y = +1 m.
        Pose2D p = RosConversions.ToMapPose(new Vector3(-4f, 0f, 8f), 0f, Scale);
        Assert.AreEqual(2.0, p.X, Tol);
        Assert.AreEqual(1.0, p.Y, Tol);
        Assert.AreEqual(0.0, p.Yaw, Tol);
    }

    [Test]
    public void MapPose_unity_yaw_90_clockwise_is_ros_yaw_minus_90()
    {
        Pose2D p = RosConversions.ToMapPose(Vector3.zero, 90f, Scale);
        Assert.AreEqual(-Math.PI / 2, p.Yaw, Tol);
    }

    [Test]
    public void MapPose_equals_the_odom_pose_of_a_robot_that_started_at_the_origin()
    {
        Vector3 at = new Vector3(3f, 0.7f, -5f);
        Pose2D map = RosConversions.ToMapPose(at, 35f, Scale);
        Pose2D odom = RosConversions.ToOdomPose(Vector3.zero, 0f, at, 35f, Scale);
        Assert.AreEqual(odom.X, map.X, Tol);
        Assert.AreEqual(odom.Y, map.Y, Tol);
        Assert.AreEqual(odom.Yaw, map.Yaw, Tol);
    }

    // ---- velocity -----------------------------------------------------

    [Test]
    public void ForwardSpeed_projects_on_heading_and_divides_by_scale()
    {
        Assert.AreEqual(0.2, RosConversions.ForwardSpeed(new Vector3(0f, 0f, 0.8f), Vector3.forward, Scale), Tol);
        Assert.AreEqual(-0.2, RosConversions.ForwardSpeed(new Vector3(0f, 0f, -0.8f), Vector3.forward, Scale), Tol);
        Assert.AreEqual(0.0, RosConversions.ForwardSpeed(new Vector3(0.8f, 0f, 0f), Vector3.forward, Scale), Tol);
    }

    [Test]
    public void YawRate_flips_unity_clockwise_to_ros_counter_clockwise()
    {
        Assert.AreEqual(-1.0, RosConversions.YawRate(new Vector3(0f, 1f, 0f)), Tol);
        Assert.AreEqual(0.5, RosConversions.YawRate(new Vector3(0f, -0.5f, 0f)), Tol);
    }

    // ---- laser scan direction -------------------------------------------

    [Test]
    public void ScanDirection_zero_angle_is_robot_forward()
    {
        AssertVector(Vector3.forward, RosConversions.ScanDirection(0f, 0.0));
        AssertVector(Vector3.right, RosConversions.ScanDirection(90f, 0.0));
    }

    [Test]
    public void ScanDirection_positive_angle_turns_left()
    {
        AssertVector(Vector3.left, RosConversions.ScanDirection(0f, Math.PI / 2));
        AssertVector(Vector3.back, RosConversions.ScanDirection(0f, Math.PI));
    }

    private static void AssertVector(Vector3 expected, Vector3 actual)
    {
        Assert.Less((expected - actual).magnitude, 1e-4f, "expected " + expected + " but was " + actual);
    }
}
