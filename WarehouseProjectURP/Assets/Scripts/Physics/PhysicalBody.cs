using UnityEngine;

/// <summary>
/// Declares an object's physics in REAL units (ADR-011): surface material, static or dynamic, and real mass.
/// Everything else (Unity mass, gravity, friction) is derived by <see cref="PhysicsStandard"/>.
///
/// Owns every collider and rigidbody below it, except those under a deeper PhysicalBody. So a static rack
/// can hold dynamic boxes that each have their own PhysicalBody.
///
/// Static: existing rigidbodies are made kinematic (they never move or fall); no Rigidbody is added.
/// Dynamic: one Rigidbody on this object with mass = real mass x 64 x (its scale volume), Earth gravity.
///
/// Not used on the robot: DiffDriveController scales the URDF masses itself, by the same rule.
/// </summary>
[DisallowMultipleComponent]
public class PhysicalBody : MonoBehaviour
{
    [Tooltip("Surface, which sets friction.")]
    public SurfaceMaterial material = SurfaceMaterial.Cardboard;

    [Tooltip("Never moves (racks, walls, floor).")]
    public bool isStatic;

    [Tooltip("REAL mass in kg at scale (1,1,1). Ignored when static. Boxes: Fragile 5, Standard 15, Heavy 30.")]
    [Min(0f)]
    public float realMassKg = 15f;

    void Awake()
    {
        Apply();
    }

    /// <summary>Applies mass, flags and friction. Safe to call again after changing the fields.</summary>
    public void Apply()
    {
        ApplyBody();
        ApplyMaterials();
    }

    /// <summary>
    /// Mass and static/dynamic flags only. These serialize, so the editor menu uses this to show the
    /// final values in the Inspector.
    /// </summary>
    public void ApplyBody()
    {
        if (isStatic)
        {
            foreach (Rigidbody rb in GetComponentsInChildren<Rigidbody>(true))
            {
                if (!Owns(rb.transform)) continue;
                rb.isKinematic = true;
                rb.useGravity = false;
            }
            return;
        }

        Rigidbody body = GetComponent<Rigidbody>();
        if (body == null) body = gameObject.AddComponent<Rigidbody>();
        body.mass = PhysicsStandard.UnityMassKg(realMassKg, transform.lossyScale);
        body.useGravity = true;
        body.isKinematic = false;
        body.drag = 0f;           // air drag is negligible for warehouse objects
        body.angularDrag = 0.05f; // Unity default; keeps resting objects calm
    }

    /// <summary>Friction. PhysicMaterials are created at runtime, so this only runs in Awake (and in tests).</summary>
    public void ApplyMaterials()
    {
        PhysicMaterial shared = PhysicsStandard.MaterialOf(material);
        foreach (Collider c in GetComponentsInChildren<Collider>(true))
        {
            if (Owns(c.transform)) c.sharedMaterial = shared;
        }
    }

    /// <summary>The nearest PhysicalBody on <paramref name="t"/> or above it (active or not), or null.</summary>
    public static PhysicalBody OwnerOf(Transform t)
    {
        for (Transform cur = t; cur != null; cur = cur.parent)
        {
            PhysicalBody body = cur.GetComponent<PhysicalBody>();
            if (body != null) return body;
        }
        return null;
    }

    private bool Owns(Transform t)
    {
        return OwnerOf(t) == this;
    }
}
