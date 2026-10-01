using UnityEngine;

/// <summary>
/// The warehouse floor tiles use BoxColliders that are essentially ZERO thick (height ~1e-13, see
/// Floor01.prefab). Physics pushes a body out of a collider along the shortest way, so anything that
/// dips even slightly below a zero-thickness plane is never pushed back up: robot casters and chassis
/// were found 1-2 cm underground, and the robot could not balance or drive.
///
/// This gives every thin, horizontal floor slab a real thickness while keeping its top surface exactly
/// where it was. It is idempotent and cheap (one pass over the box colliders), so it is safe to call
/// from several places.
/// </summary>
public static class FloorColliderFix
{
    /// <summary>Thickness given to floor slabs, in world metres.</summary>
    public const float MinThickness = 0.2f;

    private const float ThinLimit = 0.01f;       // slabs thinner than this (world) are fixed
    private const float MinSlabSize = 0.5f;      // and only if at least this wide/deep, so small boxes are left alone

    /// <summary>Thickens thin floor slabs. Returns how many colliders were changed.</summary>
    public static int Apply()
    {
        int changed = 0;
        foreach (BoxCollider box in Object.FindObjectsOfType<BoxCollider>())
        {
            if (box.isTrigger || box.GetComponentInParent<ArticulationBody>() != null) continue;

            Vector3 scale = box.transform.lossyScale;
            float worldHeight = box.size.y * Mathf.Abs(scale.y);
            float worldWidth = box.size.x * Mathf.Abs(scale.x);
            float worldDepth = box.size.z * Mathf.Abs(scale.z);

            if (worldHeight >= ThinLimit) continue;                              // already has thickness
            if (worldWidth < MinSlabSize || worldDepth < MinSlabSize) continue;  // not a slab
            if (Vector3.Dot(box.transform.up, Vector3.up) < 0.9f) continue;     // not horizontal
            if (Mathf.Abs(scale.y) < 1e-6f) continue;

            float newLocalHeight = MinThickness / Mathf.Abs(scale.y);
            Vector3 size = box.size;
            Vector3 center = box.center;
            center.y -= (newLocalHeight - size.y) * 0.5f;   // grow downwards: the top surface stays put
            size.y = newLocalHeight;
            box.size = size;
            box.center = center;
            changed++;
        }

        if (changed > 0)
        {
            Debug.Log("FloorColliderFix: gave " + changed + " zero-thickness floor collider(s) a thickness of " + MinThickness + " m.");
        }
        return changed;
    }
}
