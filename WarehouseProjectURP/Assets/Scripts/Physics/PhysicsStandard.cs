using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// The project's single physics standard (docs/ADR-011-physics-standard.md).
///
/// The Unity scene is an exact 4x model of a small real warehouse on Earth (the world ROS sees):
///   length x4 · time x1 · mass x4^3 = 64 · gravity x4 (39.24 m/s^2) · force x256 · torque/inertia x4^5.
/// With those rules, anything that happens in Unity is what would happen on Earth, read in real metres.
///
/// Authors only ever type REAL values (kg, real metres). The conversion lives here and nowhere else.
/// </summary>
public static class PhysicsStandard
{
    /// <summary>Unity units per real metre. Must equal the robot root's scale (ADR-010).</summary>
    public const float WorldScale = 4f;

    /// <summary>Standard gravity on Earth, m/s^2.</summary>
    public const float EarthGravity = 9.81f;

    /// <summary>Gravity in Unity: lengths are x4 while time is unchanged, so accelerations are x4.</summary>
    public const float UnityGravity = EarthGravity * WorldScale;

    /// <summary>TurtleBot3 Waffle Pi maximum payload (ROBOTIS spec), real kg.</summary>
    public const float RobotPayloadKg = 30f;

    private const float MassFactor = WorldScale * WorldScale * WorldScale;

    private static readonly Dictionary<SurfaceMaterial, Friction> FrictionTable = new Dictionary<SurfaceMaterial, Friction>
    {
        { SurfaceMaterial.Concrete,  new Friction(0.8f, 0.7f) },
        { SurfaceMaterial.Steel,     new Friction(0.5f, 0.4f) },
        { SurfaceMaterial.Cardboard, new Friction(0.5f, 0.4f) },
        { SurfaceMaterial.Rubber,    new Friction(1.0f, 0.9f) },
        { SurfaceMaterial.Plastic,   new Friction(0.4f, 0.3f) },
    };

    private static readonly Dictionary<SurfaceMaterial, PhysicMaterial> Materials = new Dictionary<SurfaceMaterial, PhysicMaterial>();

    /// <summary>Applies the standard gravity before any scene loads, in builds, Play mode and PlayMode tests.</summary>
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    public static void ApplyGravity()
    {
        Physics.gravity = new Vector3(0f, -UnityGravity, 0f);
    }

    /// <summary>
    /// Unity mass for an object whose REAL mass at scale (1,1,1) is <paramref name="realMassKg"/>.
    /// Scaling the object scales its mass with its volume.
    /// </summary>
    public static float UnityMassKg(float realMassKg, Vector3 lossyScale)
    {
        return realMassKg * MassFactor * Volume(lossyScale);
    }

    /// <summary>Inverse of <see cref="UnityMassKg"/>.</summary>
    public static float RealMassKg(float unityMassKg, Vector3 lossyScale)
    {
        float volume = Volume(lossyScale);
        return volume > 0f ? unityMassKg / (MassFactor * volume) : 0f;
    }

    /// <summary>Real density (kg/m^3) of a real mass filling <paramref name="unityVolume"/> Unity cubic units.</summary>
    public static float RealDensity(float realMassKg, float unityVolume)
    {
        float realVolume = unityVolume / MassFactor;
        return realVolume > 0f ? realMassKg / realVolume : float.PositiveInfinity;
    }

    /// <summary>Agreed real box masses (feature requirements Q2).</summary>
    public static float BoxRealMassKg(BoxCategory category)
    {
        switch (category)
        {
            case BoxCategory.Fragile: return 5f;
            case BoxCategory.Standard: return 15f;
            case BoxCategory.Heavy: return 30f;
            default: throw new ArgumentOutOfRangeException(nameof(category), category, null);
        }
    }

    public static Friction FrictionOf(SurfaceMaterial material)
    {
        return FrictionTable[material];
    }

    /// <summary>
    /// One shared PhysicMaterial per surface. Friction combines by Average, so a pair's friction is the mean
    /// of both surfaces. No bounce. The robot's wheels and casters keep their own verified materials (ADR-010).
    /// </summary>
    public static PhysicMaterial MaterialOf(SurfaceMaterial material)
    {
        if (Materials.TryGetValue(material, out PhysicMaterial existing) && existing != null) return existing;

        Friction f = FrictionOf(material);
        PhysicMaterial created = new PhysicMaterial("Std_" + material)
        {
            staticFriction = f.Static,
            dynamicFriction = f.Dynamic,
            bounciness = 0f,
            frictionCombine = PhysicMaterialCombine.Average,
            bounceCombine = PhysicMaterialCombine.Minimum,
            hideFlags = HideFlags.DontSave
        };
        Materials[material] = created;
        return created;
    }

    private static float Volume(Vector3 scale)
    {
        return Mathf.Abs(scale.x * scale.y * scale.z);
    }
}
