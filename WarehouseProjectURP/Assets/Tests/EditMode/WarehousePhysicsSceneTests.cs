using System.Collections.Generic;
using System.Text;
using NUnit.Framework;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// Audits Warehouse.unity against ADR-011. Uses the scene as it is open in the editor (including unsaved
/// changes); if it is not open, it is opened additively and closed again afterwards.
/// Fix failures with "Robotics > Warehouse > Apply Physics Standard", then save the scene.
/// </summary>
public class WarehousePhysicsSceneTests
{
    private const string ScenePath = "Assets/Scenes/Warehouse.unity";
    private const string RackName = "ShelvingRackRandom(Clone)";

    // Plausible real densities: packaging foam ~20 kg/m^3 up to dense goods ~3000 kg/m^3.
    private const float MinRealDensity = 20f;
    private const float MaxRealDensity = 3000f;

    private Scene scene;
    private bool openedHere;

    [OneTimeSetUp]
    public void OpenScene()
    {
        scene = SceneManager.GetSceneByPath(ScenePath);
        if (!scene.isLoaded)
        {
            scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Additive);
            openedHere = true;
        }
        Physics.SyncTransforms();
    }

    [OneTimeTearDown]
    public void CloseScene()
    {
        if (openedHere) EditorSceneManager.CloseScene(scene, true);
    }

    [Test]
    public void Project_gravity_is_the_standard()
    {
        Assert.AreEqual(-PhysicsStandard.UnityGravity, Physics.gravity.y, 1e-3f,
            "Run 'Robotics > Warehouse > Apply Physics Standard' to set the project gravity.");
    }

    [Test]
    public void Robot_scale_is_the_world_scale()
    {
        Transform robot = Robot();
        Assert.IsNotNull(robot, "no DiffDriveController in the scene");
        Assert.AreEqual(PhysicsStandard.WorldScale, robot.lossyScale.x, 1e-4f, "ADR-010/011: robot root scale");
    }

    [Test]
    public void Every_physics_object_is_owned_by_a_physical_body()
    {
        Transform robot = Robot();
        StringBuilder missing = new StringBuilder();
        int count = 0;
        foreach (Collider c in All<Collider>())
        {
            if (c.isTrigger || IsUnder(c.transform, robot)) continue;
            if (PhysicalBody.OwnerOf(c.transform) != null) continue;
            if (count++ < 10) missing.AppendLine("  collider " + PathOf(c.transform));
        }
        foreach (Rigidbody rb in All<Rigidbody>())
        {
            if (IsUnder(rb.transform, robot) || PhysicalBody.OwnerOf(rb.transform) != null) continue;
            if (count++ < 10) missing.AppendLine("  rigidbody " + PathOf(rb.transform));
        }
        Assert.AreEqual(0, count, count + " object(s) without a PhysicalBody, e.g.:\n" + missing);
    }

    [Test]
    public void Racks_are_static_steel()
    {
        int racks = 0;
        foreach (Transform t in All<Transform>())
        {
            if (t.name != RackName) continue;
            racks++;
            PhysicalBody body = t.GetComponent<PhysicalBody>();
            Assert.IsNotNull(body, PathOf(t) + " has no PhysicalBody");
            Assert.IsTrue(body.isStatic, PathOf(t) + " must be static");
            Assert.AreEqual(SurfaceMaterial.Steel, body.material, PathOf(t));
        }
        Assert.AreEqual(12, racks, "rack count (requirements R1.1)");
    }

    [Test]
    public void Dynamic_bodies_have_real_world_density()
    {
        StringBuilder bad = new StringBuilder();
        int dynamicBodies = 0;
        foreach (PhysicalBody body in All<PhysicalBody>())
        {
            if (body.isStatic) continue;
            dynamicBodies++;
            float volume = UnityVolume(body);
            float density = PhysicsStandard.RealDensity(body.realMassKg, volume);
            if (body.realMassKg <= 0f || density < MinRealDensity || density > MaxRealDensity)
            {
                bad.AppendLine("  " + PathOf(body.transform) + ": " + body.realMassKg + " kg real, " +
                               density.ToString("F0") + " kg/m^3 real");
            }
        }
        Assert.Greater(dynamicBodies, 0, "expected the shelf boxes to be dynamic PhysicalBodies");
        Assert.AreEqual(0, bad.Length, "implausible real densities:\n" + bad);
    }

    // ------------------------------------------------------------------

    private IEnumerable<T> All<T>() where T : Component
    {
        foreach (GameObject root in scene.GetRootGameObjects())
        {
            foreach (T c in root.GetComponentsInChildren<T>(true)) yield return c;
        }
    }

    private Transform Robot()
    {
        foreach (DiffDriveController d in All<DiffDriveController>()) return d.transform;
        return null;
    }

    private static bool IsUnder(Transform t, Transform root)
    {
        return root != null && t.IsChildOf(root);
    }

    /// <summary>Volume of the colliders this body owns (world AABB, so rotated boxes are slightly overestimated).</summary>
    private static float UnityVolume(PhysicalBody body)
    {
        float volume = 0f;
        foreach (Collider c in body.GetComponentsInChildren<Collider>(true))
        {
            if (c.isTrigger || PhysicalBody.OwnerOf(c.transform) != body) continue;
            Vector3 s = c.bounds.size;
            volume += s.x * s.y * s.z;
        }
        return volume;
    }

    private static string PathOf(Transform t)
    {
        string path = t.name;
        while (t.parent != null)
        {
            t = t.parent;
            path = t.name + "/" + path;
        }
        return path;
    }
}
