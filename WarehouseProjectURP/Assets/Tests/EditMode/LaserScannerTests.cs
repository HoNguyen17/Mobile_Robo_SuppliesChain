using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Raycast LIDAR against a small test world built in the open scene (removed again in TearDown).
/// Robot scale 4: 1 robot metre = 4 Unity units (ADR-010).
/// </summary>
public class LaserScannerTests
{
    private const float Scale = 4f;
    private const int Beams = 360;
    private const float RangeMinM = 0.12f;
    private const float RangeMaxM = 3.5f;
    // Far above the warehouse, so nothing in the open scene can interfere.
    private static readonly Vector3 Origin = new Vector3(0f, 1000f, 0f);

    private readonly List<GameObject> created = new List<GameObject>();
    private Transform robot;

    [SetUp]
    public void SetUp()
    {
        robot = Track(new GameObject("test_robot")).transform;
        robot.position = Origin;
    }

    [TearDown]
    public void TearDown()
    {
        foreach (GameObject go in created) Object.DestroyImmediate(go);
        created.Clear();
    }

    [Test]
    public void Measures_wall_ahead_in_robot_metres()
    {
        Box(Origin + new Vector3(0f, 0f, 8f), new Vector3(20f, 2f, 0.2f)); // near face at z = 7.9

        float[] ranges = Scan(0f);

        Assert.AreEqual(Beams, ranges.Length);
        Assert.AreEqual(7.9f / Scale, ranges[0], 1e-3f);
    }

    [Test]
    public void Beam_90_looks_left()
    {
        Box(Origin + new Vector3(-6f, 0f, 0f), new Vector3(0.2f, 2f, 20f)); // wall on Unity -X = robot left

        float[] ranges = Scan(0f);

        Assert.AreEqual(5.9f / Scale, ranges[90], 1e-3f);
        Assert.IsTrue(float.IsPositiveInfinity(ranges[270]), "nothing on the right");
    }

    [Test]
    public void No_return_is_positive_infinity()
    {
        float[] ranges = Scan(0f);
        Assert.IsTrue(float.IsPositiveInfinity(ranges[0]));
        Assert.IsTrue(float.IsPositiveInfinity(ranges[180]));
    }

    [Test]
    public void Objects_beyond_max_range_are_not_seen()
    {
        Box(Origin + new Vector3(0f, 0f, 20f), new Vector3(20f, 2f, 0.2f)); // 4.95 m > 3.5 m

        Assert.IsTrue(float.IsPositiveInfinity(Scan(0f)[0]));
    }

    [Test]
    public void Robot_own_parts_are_ignored()
    {
        GameObject part = Box(Origin + new Vector3(0f, 0f, 0.8f), new Vector3(0.5f, 0.5f, 0.5f));
        part.transform.SetParent(robot, true);
        Box(Origin + new Vector3(0f, 0f, 8f), new Vector3(20f, 2f, 0.2f));

        Assert.AreEqual(7.9f / Scale, Scan(0f)[0], 1e-3f);
    }

    [Test]
    public void Triggers_are_ignored()
    {
        Box(Origin + new Vector3(0f, 0f, 4f), new Vector3(2f, 2f, 0.2f)).GetComponent<Collider>().isTrigger = true;

        Assert.IsTrue(float.IsPositiveInfinity(Scan(0f)[0]));
    }

    [Test]
    public void Follows_robot_heading()
    {
        Box(Origin + new Vector3(8f, 0f, 0f), new Vector3(0.2f, 2f, 20f)); // wall on Unity +X

        Assert.AreEqual(7.9f / Scale, Scan(90f)[0], 1e-3f);
    }

    private float[] Scan(float yawDeg)
    {
        Physics.SyncTransforms();
        var scanner = new LaserScanner(Beams, RangeMinM, RangeMaxM, robot, Physics.DefaultRaycastLayers);
        return scanner.Scan(Origin, yawDeg, Scale);
    }

    private GameObject Box(Vector3 centre, Vector3 size)
    {
        GameObject go = Track(GameObject.CreatePrimitive(PrimitiveType.Cube));
        go.transform.position = centre;
        go.transform.localScale = size;
        return go;
    }

    private GameObject Track(GameObject go)
    {
        created.Add(go);
        return go;
    }
}
