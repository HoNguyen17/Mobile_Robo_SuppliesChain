using UnityEngine;

/// <summary>
/// A 2D raycast LIDAR. Beams fan out in the horizontal plane at the scanner's height, using only the
/// robot's yaw, so a small pitch or roll of the robot never makes the beams hit the floor.
///
/// Ranges are returned in ROBOT metres (world distance / robot scale, ADR-010), counter-clockwise from
/// robot forward, like sensor_msgs/LaserScan with angle_min = 0. No return within range_max is +Infinity
/// (REP-117). The robot's own colliders, and anything parented to the robot (a carried item), are ignored.
/// </summary>
public sealed class LaserScanner
{
    private const int MaxHitsPerBeam = 16;

    private readonly Transform ignoreRoot;
    private readonly int layerMask;
    private readonly RaycastHit[] hits = new RaycastHit[MaxHitsPerBeam];

    public int BeamCount { get; }
    public float RangeMinM { get; }
    public float RangeMaxM { get; }
    public float AngleIncrement => 2f * Mathf.PI / BeamCount;

    public LaserScanner(int beamCount, float rangeMinM, float rangeMaxM, Transform ignoreRoot, int layerMask)
    {
        BeamCount = Mathf.Max(1, beamCount);
        RangeMinM = rangeMinM;
        RangeMaxM = rangeMaxM;
        this.ignoreRoot = ignoreRoot;
        this.layerMask = layerMask;
    }

    /// <summary>One full sweep from <paramref name="origin"/> (world), robot heading <paramref name="yawDeg"/> (Unity yaw).</summary>
    public float[] Scan(Vector3 origin, float yawDeg, float robotScale)
    {
        float[] ranges = new float[BeamCount];
        float maxWorld = RangeMaxM * robotScale;
        float increment = AngleIncrement;

        for (int i = 0; i < BeamCount; i++)
        {
            Vector3 dir = RosConversions.ScanDirection(yawDeg, i * increment);
            float nearest = NearestHit(origin, dir, maxWorld);
            ranges[i] = float.IsPositiveInfinity(nearest) ? float.PositiveInfinity : nearest / robotScale;
        }
        return ranges;
    }

    private float NearestHit(Vector3 origin, Vector3 dir, float maxWorld)
    {
        int count = Physics.RaycastNonAlloc(origin, dir, hits, maxWorld, layerMask, QueryTriggerInteraction.Ignore);
        float nearest = float.PositiveInfinity;
        for (int h = 0; h < count; h++)
        {
            if (hits[h].distance >= nearest) continue;
            if (ignoreRoot != null && hits[h].collider.transform.IsChildOf(ignoreRoot)) continue;
            nearest = hits[h].distance;
        }
        return nearest;
    }
}
