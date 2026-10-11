using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using Debug = UnityEngine.Debug;

/// <summary>
/// Builds the static map once and publishes it on /map (nav_msgs/OccupancyGrid) at 1 Hz (ADR-012).
///
/// What blocks a cell is what blocks the physical robot: a non-trigger collider that belongs to a STATIC
/// PhysicalBody (walls, racks, stations: ADR-011) and that reaches into the height band between
/// <see cref="bandBottomM"/> and <see cref="bandTopM"/> above the robot base. Dynamic boxes, the robot itself and
/// ceiling fixtures are left out. The grid is raw: it is NOT inflated here, the planner does that.
///
/// The grid is in robot metres in the ROS "map" frame (<see cref="MapGridMath"/>); it is rebuilt at every Play,
/// so changing the scale does not invalidate a saved map. The build is spread over frames.
///
/// Put this on the robot's ROOT GameObject, next to DiffDriveController.
/// </summary>
[RequireComponent(typeof(DiffDriveController))]
[DisallowMultipleComponent]
public class WarehouseMapPublisher : MonoBehaviour
{
    [Header("ROS")]
    public string topic = "/map";
    public string frameId = "map";
    public float publishHz = 1f;

    [Header("Grid (robot metres)")]
    [Tooltip("Cell size. 0.05 m is the usual TurtleBot3 map resolution and fine enough for a 0.10 m clearance.")]
    public float cellSizeM = 0.05f;

    [Tooltip("Free space added around the outermost collider, so paths can go around it.")]
    public float paddingM = 1f;

    [Header("Height band above the robot base (robot metres)")]
    [Tooltip("Colliders that end below this are floor, not obstacles.")]
    public float bandBottomM = 0.03f;

    [Tooltip("Colliders that start above this are overhead (lights, ceiling), the robot passes under them.")]
    public float bandTopM = 1.0f;

    [Header("Timing")]
    [Tooltip("Seconds to wait after Play before building, so the warehouse has finished spawning and settling.")]
    public float startDelay = 1f;

    [Tooltip("Time per frame spent on the build. The rest of the frame stays free for physics and rendering.")]
    public float buildBudgetMs = 8f;

    private const int MaxCells = 4000000;
    private const int HitBufferSize = 64;

    private ROSConnection ros;
    private DiffDriveController drive;
    private PublishTimer timer;
    private uint seq;

    public bool IsBuilt { get; private set; }
    public MapGridSpec Spec { get; private set; }

    /// <summary>The finished grid, data[j * Spec.Width + i]: 0 free, 100 blocked. Null until <see cref="IsBuilt"/>.</summary>
    public sbyte[] Data { get; private set; }

    IEnumerator Start()
    {
        drive = GetComponent<DiffDriveController>();
        timer = new PublishTimer(publishHz);
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OccupancyGridMsg>(topic);

        yield return new WaitForSeconds(startDelay);
        yield return BuildAsync();
    }

    void Update()
    {
        if (!IsBuilt || !timer.Tick(Time.timeAsDouble)) return;
        Publish();
    }

    /// <summary>Builds the whole map before returning. For tests; the game uses the frame-by-frame build.</summary>
    public void BuildNow()
    {
        if (drive == null) drive = GetComponent<DiffDriveController>();
        IEnumerator build = BuildAsync();
        while (build.MoveNext()) { }
    }

    private IEnumerator BuildAsync()
    {
        float scale = transform.lossyScale.x;
        float baseY = drive.BaseBody.position.y;
        float bottom = baseY + bandBottomM * scale;
        float top = baseY + bandTopM * scale;

        Physics.SyncTransforms();
        Bounds bounds;
        HashSet<Collider> blockers = CollectBlockers(bottom, top, out bounds);
        if (blockers.Count == 0)
        {
            Debug.LogError("WarehouseMapPublisher: no static collider reaches the height band. Did 'Robotics > Warehouse > " +
                           "Apply Physics Standard' run, so the racks and walls have a static PhysicalBody?", this);
            yield break;
        }

        MapGridSpec spec = MapGridMath.FromUnityBounds(bounds.min.x, bounds.min.z, bounds.max.x, bounds.max.z,
                                                       scale, cellSizeM, paddingM);
        if (spec.Count > MaxCells)
        {
            Debug.LogError("WarehouseMapPublisher: the map would have " + spec.Count + " cells (limit " + MaxCells +
                           "). Check the cell size and the scale.", this);
            yield break;
        }

        sbyte[] data = new sbyte[spec.Count];
        Vector3 half = new Vector3(cellSizeM * scale * 0.5f, (top - bottom) * 0.5f, cellSizeM * scale * 0.5f);
        float middleY = (top + bottom) * 0.5f;
        Collider[] hits = new Collider[HitBufferSize];
        Stopwatch slice = Stopwatch.StartNew();
        Stopwatch total = Stopwatch.StartNew();

        for (int j = 0; j < spec.Height; j++)
        {
            int row = j;
            MapGridMath.FillRow(spec, data, row, (i, jj) =>
                IsBlocked(MapGridMath.CellCentreUnity(spec, i, jj, middleY), half, hits, blockers));

            if (slice.ElapsedMilliseconds >= buildBudgetMs)
            {
                yield return null;
                slice.Restart();
            }
        }

        int occupied = 0;
        for (int k = 0; k < data.Length; k++) if (data[k] != MapGridMath.Free) occupied++;

        Spec = spec;
        Data = data;
        IsBuilt = true;
        Debug.Log("WarehouseMapPublisher: built " + spec.Width + " x " + spec.Height + " cells of " + spec.Cell.ToString("F2") +
                  " m (origin " + spec.OriginX.ToString("F2") + ", " + spec.OriginY.ToString("F2") + "), " +
                  (100.0 * occupied / data.Length).ToString("F1") + "% blocked, " + blockers.Count + " colliders, " +
                  total.ElapsedMilliseconds + " ms.", this);
    }

    /// <summary>Static, solid colliders that reach into the height band. Also returns their combined bounds.</summary>
    private HashSet<Collider> CollectBlockers(float bottom, float top, out Bounds bounds)
    {
        HashSet<Collider> blockers = new HashSet<Collider>();
        bounds = new Bounds();
        bool first = true;
        foreach (Collider c in FindObjectsOfType<Collider>())
        {
            if (!c.enabled || c.isTrigger || !c.gameObject.activeInHierarchy) continue;
            if (c.transform.IsChildOf(transform)) continue; // the robot itself

            PhysicalBody owner = PhysicalBody.OwnerOf(c.transform);
            if (owner == null || !owner.isStatic) continue;

            Bounds b = c.bounds;
            if (b.max.y < bottom || b.min.y > top) continue;

            blockers.Add(c);
            if (first) { bounds = b; first = false; }
            else bounds.Encapsulate(b);
        }
        return blockers;
    }

    private static bool IsBlocked(Vector3 centre, Vector3 half, Collider[] hits, HashSet<Collider> blockers)
    {
        int n = Physics.OverlapBoxNonAlloc(centre, half, hits, Quaternion.identity, Physics.AllLayers,
                                           QueryTriggerInteraction.Ignore);
        if (n >= hits.Length) return true; // crowded cell: assume the worst
        for (int k = 0; k < n; k++)
        {
            if (blockers.Contains(hits[k])) return true;
        }
        return false;
    }

    private void Publish()
    {
        MapGridSpec spec = Spec;
        OccupancyGridMsg msg = new OccupancyGridMsg();
        msg.header = new HeaderMsg(seq++, RosConversions.ToRosTime(Time.timeAsDouble), frameId);
        msg.info.resolution = (float)spec.Cell;
        msg.info.width = (uint)spec.Width;
        msg.info.height = (uint)spec.Height;
        msg.info.origin = new PoseMsg(new PointMsg(spec.OriginX, spec.OriginY, 0.0), new QuaternionMsg(0.0, 0.0, 0.0, 1.0));
        msg.data = Data;
        ros.Publish(topic, msg);
    }
}
