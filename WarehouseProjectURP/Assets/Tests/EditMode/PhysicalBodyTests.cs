using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

/// <summary>PhysicalBody applies ADR-011 to its own colliders and rigidbodies, and to children it owns.</summary>
public class PhysicalBodyTests
{
    private readonly List<GameObject> created = new List<GameObject>();

    [TearDown]
    public void TearDown()
    {
        foreach (GameObject go in created) Object.DestroyImmediate(go);
        created.Clear();
    }

    [Test]
    public void Dynamic_body_gets_scaled_mass_gravity_and_material()
    {
        GameObject box = Cube("box");
        PhysicalBody body = box.AddComponent<PhysicalBody>();
        body.material = SurfaceMaterial.Cardboard;
        body.realMassKg = 15f;

        body.Apply();

        Rigidbody rb = box.GetComponent<Rigidbody>();
        Assert.IsNotNull(rb, "a dynamic body gets a Rigidbody");
        Assert.AreEqual(960f, rb.mass, 1e-2f);
        Assert.IsTrue(rb.useGravity);
        Assert.IsFalse(rb.isKinematic);
        Assert.AreSame(PhysicsStandard.MaterialOf(SurfaceMaterial.Cardboard), box.GetComponent<Collider>().sharedMaterial);
    }

    [Test]
    public void Scaled_dynamic_body_gets_more_mass()
    {
        GameObject box = Cube("big_box");
        box.transform.localScale = Vector3.one * 2f;
        PhysicalBody body = box.AddComponent<PhysicalBody>();
        body.realMassKg = 15f;

        body.Apply();

        Assert.AreEqual(960f * 8f, box.GetComponent<Rigidbody>().mass, 1e-1f);
    }

    [Test]
    public void Static_body_freezes_its_rigidbodies_without_adding_one()
    {
        GameObject rack = Track(new GameObject("rack"));
        GameObject leg = Cube("leg");
        leg.transform.SetParent(rack.transform);
        Rigidbody legRb = leg.AddComponent<Rigidbody>();
        PhysicalBody body = rack.AddComponent<PhysicalBody>();
        body.isStatic = true;
        body.material = SurfaceMaterial.Steel;

        body.Apply();

        Assert.IsNull(rack.GetComponent<Rigidbody>(), "static bodies never get a new Rigidbody");
        Assert.IsTrue(legRb.isKinematic);
        Assert.IsFalse(legRb.useGravity);
        Assert.AreSame(PhysicsStandard.MaterialOf(SurfaceMaterial.Steel), leg.GetComponent<Collider>().sharedMaterial);
    }

    [Test]
    public void Child_with_its_own_physical_body_is_left_alone()
    {
        GameObject rack = Track(new GameObject("rack"));
        PhysicalBody rackBody = rack.AddComponent<PhysicalBody>();
        rackBody.isStatic = true;
        rackBody.material = SurfaceMaterial.Steel;

        GameObject box = Cube("box_on_rack");
        box.transform.SetParent(rack.transform);
        Rigidbody boxRb = box.AddComponent<Rigidbody>();
        PhysicalBody boxBody = box.AddComponent<PhysicalBody>();
        boxBody.material = SurfaceMaterial.Cardboard;
        boxBody.realMassKg = 5f;

        rackBody.Apply();
        boxBody.Apply();

        Assert.IsFalse(boxRb.isKinematic, "the rack must not freeze the box it holds");
        Assert.AreEqual(320f, boxRb.mass, 1e-2f);
        Assert.AreSame(PhysicsStandard.MaterialOf(SurfaceMaterial.Cardboard), box.GetComponent<Collider>().sharedMaterial);
    }

    [Test]
    public void Apply_is_idempotent()
    {
        GameObject box = Cube("box");
        PhysicalBody body = box.AddComponent<PhysicalBody>();
        body.realMassKg = 15f;

        body.Apply();
        body.Apply();

        Assert.AreEqual(1, box.GetComponents<Rigidbody>().Length);
        Assert.AreEqual(960f, box.GetComponent<Rigidbody>().mass, 1e-2f);
    }

    private GameObject Cube(string name)
    {
        GameObject go = Track(GameObject.CreatePrimitive(PrimitiveType.Cube));
        go.name = name;
        go.transform.position = new Vector3(0f, 2000f, 0f); // far from the warehouse
        return go;
    }

    private GameObject Track(GameObject go)
    {
        created.Add(go);
        return go;
    }
}
