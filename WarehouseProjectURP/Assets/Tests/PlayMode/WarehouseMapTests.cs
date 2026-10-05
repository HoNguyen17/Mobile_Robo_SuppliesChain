using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
#if UNITY_EDITOR
using UnityEditor.SceneManagement;
#endif
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

/// <summary>
/// The static /map built from the real Warehouse scene: it must agree with the physical world (ADR-012).
/// Builds the map with WarehouseMapPublisher.BuildNow, so no ROS connection is needed.
/// </summary>
public class WarehouseMapTests
{
    private const string ScenePath = "Assets/Scenes/Warehouse.unity";
    private const string RackName = "ShelvingRackRandom(Clone)";

    [UnityTest]
    [Timeout(300000)]
    public IEnumerator Map_agrees_with_the_physical_warehouse()
    {
#if UNITY_EDITOR
        yield return EditorSceneManager.LoadSceneAsyncInPlayMode(ScenePath, new LoadSceneParameters(LoadSceneMode.Single));
#else
        yield break;
#endif
        yield return null;
        yield return new WaitForSeconds(3f); // let the robot settle under gravity, as RobotDriveTests does

        DiffDriveController drive = Object.FindObjectOfType<DiffDriveController>();
        Assert.IsNotNull(drive, "no DiffDriveController in the scene");
        WarehouseMapPublisher publisher = drive.GetComponent<WarehouseMapPublisher>();
        if (publisher == null) publisher = drive.gameObject.AddComponent<WarehouseMapPublisher>();

        publisher.BuildNow();

        Assert.IsTrue(publisher.IsBuilt, "the map was not built (see the Console for the reason)");
        MapGridSpec spec = publisher.Spec;
        sbyte[] data = publisher.Data;
        Assert.AreEqual(spec.Count, data.Length);
        Assert.AreEqual(publisher.cellSizeM, spec.Cell, 1e-6);
        Assert.AreEqual(PhysicsStandard.WorldScale, spec.Scale, 1e-4f);

        int blocked = 0;
        foreach (sbyte cell in data) if (cell == MapGridMath.Occupied) blocked++;
        float share = blocked / (float)data.Length;
        Debug.Log("[MapTest] " + spec.Width + " x " + spec.Height + " cells, " + (100f * share).ToString("F1") + "% blocked");
        Assert.Greater(share, 0.005f, "almost nothing is blocked: racks and walls are missing");
        Assert.Less(share, 0.60f, "most of the map is blocked");

        AssertFree(spec, data, RobotPosition(drive), "the robot's own cell");

        TurtleBotNavigator navigator = drive.GetComponent<TurtleBotNavigator>();
        if (navigator != null && navigator.homePoint != null)
        {
            AssertFree(spec, data, navigator.homePoint.position, "the home point");
        }

        List<Transform> racks = FindRacks();
        Assert.Greater(racks.Count, 0, "no rack named '" + RackName + "' in the scene");
        foreach (Transform rack in racks)
        {
            Assert.Greater(BlockedCellsInside(spec, data, rack), 0, "no blocked cell under " + rack.name + " at " + rack.position);
        }
    }

    private static Vector3 RobotPosition(DiffDriveController drive)
    {
        return drive.BaseBody.position;
    }

    private static void AssertFree(MapGridSpec spec, sbyte[] data, Vector3 unityPosition, string what)
    {
        Pose2D p = RosConversions.ToMapPose(unityPosition, 0f, spec.Scale);
        int index = MapGridMath.CellIndex(spec, p.X, p.Y);
        Assert.GreaterOrEqual(index, 0, what + " is outside the map");
        Assert.AreEqual(MapGridMath.Free, data[index], what + " is blocked in the map (x=" + p.X.ToString("F2") + " y=" + p.Y.ToString("F2") + ")");
    }

    private static List<Transform> FindRacks()
    {
        List<Transform> racks = new List<Transform>();
        foreach (Transform t in Object.FindObjectsOfType<Transform>())
        {
            if (t.name == RackName) racks.Add(t);
        }
        return racks;
    }

    /// <summary>Counts blocked cells whose centre lies inside the combined bounds of the rack's colliders.</summary>
    private static int BlockedCellsInside(MapGridSpec spec, sbyte[] data, Transform rack)
    {
        Bounds b = default;
        bool any = false;
        foreach (Collider c in rack.GetComponentsInChildren<Collider>())
        {
            if (c.isTrigger) continue;
            if (!any) { b = c.bounds; any = true; } else b.Encapsulate(c.bounds);
        }
        if (!any) return 0;

        int count = 0;
        for (int j = 0; j < spec.Height; j++)
        {
            for (int i = 0; i < spec.Width; i++)
            {
                if (data[j * spec.Width + i] != MapGridMath.Occupied) continue;
                Vector3 centre = MapGridMath.CellCentreUnity(spec, i, j, b.center.y);
                if (b.Contains(centre)) count++;
            }
        }
        return count;
    }
}
