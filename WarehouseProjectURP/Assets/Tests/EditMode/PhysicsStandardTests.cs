using NUnit.Framework;
using UnityEngine;

/// <summary>
/// ADR-011: the Unity scene is an exact 4x model of a small real warehouse on Earth.
/// Length x4, time x1, mass x64, gravity x4.
/// </summary>
public class PhysicsStandardTests
{
    [Test]
    public void World_scale_is_four()
    {
        Assert.AreEqual(4f, PhysicsStandard.WorldScale);
    }

    [Test]
    public void Unity_gravity_is_earth_gravity_times_world_scale()
    {
        Assert.AreEqual(9.81f, PhysicsStandard.EarthGravity, 1e-6f);
        Assert.AreEqual(39.24f, PhysicsStandard.UnityGravity, 1e-4f);
    }

    [Test]
    public void Unity_mass_is_real_mass_times_scale_cubed()
    {
        Assert.AreEqual(960f, PhysicsStandard.UnityMassKg(15f, Vector3.one), 1e-3f);
    }

    [Test]
    public void Scaling_an_object_scales_its_mass_with_volume()
    {
        Assert.AreEqual(960f * 8f, PhysicsStandard.UnityMassKg(15f, Vector3.one * 2f), 1e-2f);
        Assert.AreEqual(960f * 2f, PhysicsStandard.UnityMassKg(15f, new Vector3(1f, 2f, 1f)), 1e-2f);
    }

    [Test]
    public void Negative_scale_does_not_give_negative_mass()
    {
        Assert.AreEqual(960f, PhysicsStandard.UnityMassKg(15f, new Vector3(-1f, 1f, 1f)), 1e-3f);
    }

    [Test]
    public void Real_mass_round_trips()
    {
        float unity = PhysicsStandard.UnityMassKg(30f, Vector3.one);
        Assert.AreEqual(30f, PhysicsStandard.RealMassKg(unity, Vector3.one), 1e-4f);
    }

    [Test]
    public void Box_categories_have_agreed_real_masses()
    {
        Assert.AreEqual(5f, PhysicsStandard.BoxRealMassKg(BoxCategory.Fragile));
        Assert.AreEqual(15f, PhysicsStandard.BoxRealMassKg(BoxCategory.Standard));
        Assert.AreEqual(30f, PhysicsStandard.BoxRealMassKg(BoxCategory.Heavy));
    }

    [Test]
    public void Heaviest_box_is_within_waffle_pi_payload()
    {
        Assert.LessOrEqual(PhysicsStandard.BoxRealMassKg(BoxCategory.Heavy), PhysicsStandard.RobotPayloadKg);
    }

    [Test]
    public void Every_surface_has_sane_friction()
    {
        foreach (SurfaceMaterial m in System.Enum.GetValues(typeof(SurfaceMaterial)))
        {
            Friction f = PhysicsStandard.FrictionOf(m);
            Assert.That(f.Static, Is.InRange(0f, 2f), m + " static");
            Assert.That(f.Dynamic, Is.InRange(0f, 2f), m + " dynamic");
            Assert.GreaterOrEqual(f.Static, f.Dynamic, m + ": static friction must be >= dynamic");
        }
    }

    [Test]
    public void Materials_are_shared_and_configured()
    {
        PhysicMaterial a = PhysicsStandard.MaterialOf(SurfaceMaterial.Cardboard);
        PhysicMaterial b = PhysicsStandard.MaterialOf(SurfaceMaterial.Cardboard);
        Assert.AreSame(a, b, "one shared instance per surface");
        Assert.AreEqual(PhysicsStandard.FrictionOf(SurfaceMaterial.Cardboard).Static, a.staticFriction, 1e-6f);
        Assert.AreEqual(PhysicMaterialCombine.Average, a.frictionCombine);
        Assert.AreEqual(0f, a.bounciness);
    }

    [Test]
    public void Real_density_is_checked_in_real_units()
    {
        // A 1.13 m Unity box is a 0.28 m real box: 15 kg in 0.0211 m^3 is about 710 kg/m^3.
        float density = PhysicsStandard.RealDensity(15f, 1.130695f * 1.060026f * 1.130695f);
        Assert.AreEqual(710f, density, 15f);
    }
}
