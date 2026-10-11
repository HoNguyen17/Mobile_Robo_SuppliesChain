using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// UNITY-ONLY TEST MODE. Plans a path with A* inside Unity and drives the TurtleBot3 along it
/// through <see cref="DiffDriveController"/>. Ported from the old CubeCarNavigator.
///
/// The graded path does NOT use this: once ROS is connected, ros1/turtlebot_control plans and sends /cmd_vel
/// to the same DiffDriveController (docs/ADR-012, docs/ADR-015). Switch Auto Start off then, and
/// call GoToTarget() yourself when you want the Unity-only mission.
///
/// Two layers:
///   1. GLOBAL PLANNER - occupancy grid of the floor (shelves inflated by the robot radius) + A*,
///                       then the zig-zag is straightened into a few corners.
///   2. LOCAL FOLLOWER - turns the next corner into a velocity command (linear m/s, angular rad/s):
///                       pivots on the spot for sharp turns, drives and steers for gentle ones,
///                       and slows down near the goal.
///
/// Mission: Idle -> DrivingToTarget -> WaitingAtTarget -> DrivingHome -> Idle
///
/// Put this on the robot's ROOT GameObject, next to DiffDriveController.
/// Tunables ending in "M" are in ROBOT metres (Waffle Pi: ~0.28 m wide, 0.26 m/s top speed) and are
/// multiplied by the robot root's scale, so the robot can be scaled up (e.g. 4x) to suit the warehouse.
/// </summary>
[RequireComponent(typeof(DiffDriveController))]
public class TurtleBotNavigator : MonoBehaviour
{
    public enum NavState
    {
        Idle,             // parked, doing nothing
        DrivingToTarget,  // heading to the shelf
        WaitingAtTarget,  // arrived at the shelf, dwelling (simulated pick-up)
        DrivingHome       // returning to the home/drop-off point
    }

    [Header("Destinations")]
    [Tooltip("Drag the target shelf (or any Transform) here.")]
    public Transform target;

    [Tooltip("Where the robot returns to. Leave empty to return to the spawn position. " +
             "Do NOT make this a child of the robot.")]
    public Transform homePoint;

    [Header("Mission Behaviour")]
    public bool autoStart = true;
    public bool autoReturnHome = true;

    [Tooltip("Seconds to wait at the shelf before returning (simulated pick-up time).")]
    public float waitTimeAtTarget = 2f;

    [Header("Driving (metres, seconds)")]
    [Tooltip("Cruise speed in ROBOT m/s (world speed = this x robot scale). Capped by DiffDriveController.maxLinearSpeed (0.26 for the Waffle Pi).")]
    public float cruiseSpeed = 0.26f;

    [Tooltip("How strongly the robot steers toward the next corner (rad/s per rad of heading error).")]
    public float headingGain = 2f;

    [Tooltip("Stop once within this distance of the free spot in front of the target shelf.")]
    public float stoppingDistanceM = 0.15f;

    [Tooltip("Stop once within this distance of the home point.")]
    public float homeStoppingDistanceM = 0.2f;

    [Tooltip("Start slowing down once within this distance of the final goal.")]
    public float slowDownDistanceM = 0.6f;

    [Tooltip("A path corner counts as 'reached' within this distance.")]
    public float waypointToleranceM = 0.15f;

    [Tooltip("If the next corner is more than this many degrees off, stop and turn on the spot first.")]
    [Range(10f, 170f)]
    public float pivotAngle = 45f;

    [Header("Path Planning (Occupancy Grid + A*)")]
    [Tooltip("Every Renderer under a GameObject with one of these tags is an obstacle.")]
    public string[] obstacleTags = { "Shelf" };

    [Tooltip("Optional: extra parent objects whose child Renderers are also obstacles (e.g. Walls).")]
    public Transform[] extraObstacleRoots;

    [Tooltip("Grid cell size in metres. Smaller = more precise but slower to plan.")]
    public float cellSizeM = 0.1f;

    [Tooltip("Safety margin around obstacles. The Waffle Pi reaches 0.257 m from its turning centre, " +
             "so keep this above that. Raise if it clips shelves; lower if 'no path' appears.")]
    public float robotRadiusM = 0.3f;

    [Tooltip("Renderers that start higher than this above the robot base are ignored (ceiling lights etc.).")]
    public float overheadClearanceM = 1.0f;

    [Tooltip("Extra free space around everything, so paths can go around the outermost shelves.")]
    public float gridPaddingM = 2f;

    [Tooltip("How far to search for a free cell when the robot/target/home sits inside a blocked area.")]
    public float snapDistanceM = 3f;

    [Header("Debug")]
    [Tooltip("T = go to target, H = return home, S = stop.")]
    public bool enableDebugKeys = true;

    [Tooltip("Draw blocked cells (red). Can be slow on large warehouses with small cells.")]
    public bool drawGrid;

    [Tooltip("Draw the planned path (yellow).")]
    public bool drawPath = true;

    /// <summary>Current state, readable from other scripts.</summary>
    public NavState State { get; private set; } = NavState.Idle;

    private const int MaxGridCells = 2000000;

    // Working values in WORLD units = the '...M' tunables (robot metres) x robot scale.
    private float unitScale = 1f;
    private float stoppingDistance;
    private float homeStoppingDistance;
    private float slowDownDistance;
    private float waypointTolerance;
    private float cellSize;
    private float robotRadius;
    private float overheadClearance;
    private float gridPadding;
    private float snapDistance;

    private DiffDriveController drive;
    private Transform robotBase; // moving body (articulation root), resolved from DiffDriveController
    private Vector3 spawnPosition;
    private float waitTimer;

    // Current planned path (world positions)
    private Vector3[] corners = new Vector3[0];
    private int cornerIndex;

    // Occupancy grid (rebuilt on every plan)
    private bool[] blocked;
    private int gridW;
    private int gridH;
    private Vector2 gridOrigin; // world (x, z) of the grid's minimum corner
    private float gridY;        // height used for drawing / corner points

    void Awake()
    {
        drive = GetComponent<DiffDriveController>();
        robotBase = drive.BaseBody; // the articulation root: the part that really moves

        // All '...M' tunables are in ROBOT metres. The robot may be scaled up in the scene
        // (e.g. 4x to fit the warehouse), so convert them to world units once here.
        unitScale = transform.lossyScale.x;
        stoppingDistance = stoppingDistanceM * unitScale;
        homeStoppingDistance = homeStoppingDistanceM * unitScale;
        slowDownDistance = slowDownDistanceM * unitScale;
        waypointTolerance = waypointToleranceM * unitScale;
        cellSize = cellSizeM * unitScale;
        robotRadius = robotRadiusM * unitScale;
        overheadClearance = overheadClearanceM * unitScale;
        gridPadding = gridPaddingM * unitScale;
        snapDistance = snapDistanceM * unitScale;
        Debug.Log("TurtleBotNavigator: robot scale " + unitScale + " -> grid cell " + cellSize +
                  " units, safety radius " + robotRadius + " units.", this);
        spawnPosition = robotBase.position;
    }

    IEnumerator Start()
    {
        if (autoStart)
        {
            // Wait one frame so anything that spawns the warehouse at startup has finished.
            yield return null;
            GoToTarget();
        }
    }

    // ------------------------------------------------------------------
    // Public API
    // ------------------------------------------------------------------

    /// <summary>Plan a path to the target shelf and start driving.</summary>
    public void GoToTarget()
    {
        if (target == null)
        {
            Debug.LogWarning("TurtleBotNavigator: no target assigned.", this);
            return;
        }
        State = PlanPath(target.position) ? NavState.DrivingToTarget : NavState.Idle;
    }

    /// <summary>Plan a path back to the home point and start driving.</summary>
    public void ReturnHome()
    {
        State = PlanPath(GetHomePosition()) ? NavState.DrivingHome : NavState.Idle;
    }

    /// <summary>Stop where we are.</summary>
    public void Stop()
    {
        State = NavState.Idle;
        drive.Stop();
    }

    /// <summary>Switch to a new shelf and drive there.</summary>
    public void SetTargetAndGo(Transform newTarget)
    {
        target = newTarget;
        GoToTarget();
    }

    // ------------------------------------------------------------------

    void Update()
    {
        if (enableDebugKeys)
        {
            if (Input.GetKeyDown(KeyCode.T)) GoToTarget();
            if (Input.GetKeyDown(KeyCode.H)) ReturnHome();
            if (Input.GetKeyDown(KeyCode.S)) Stop();
        }

        switch (State)
        {
            case NavState.DrivingToTarget:
                if (FollowPath(stoppingDistance))
                {
                    waitTimer = waitTimeAtTarget;
                    State = autoReturnHome ? NavState.WaitingAtTarget : NavState.Idle;
                }
                break;

            case NavState.WaitingAtTarget:
                drive.Stop();
                waitTimer -= Time.deltaTime;
                if (waitTimer <= 0f) ReturnHome();
                break;

            case NavState.DrivingHome:
                if (FollowPath(homeStoppingDistance)) State = NavState.Idle;
                break;
        }

        // Idle: send nothing; the controller's command timeout stops the wheels.
        // That also leaves the robot free for DiffDriveController's keyboard teleop.
    }

    // ------------------------------------------------------------------
    // Layer 1: global planning (occupancy grid + A*)
    // ------------------------------------------------------------------

    private Vector3 GetHomePosition()
    {
        return homePoint != null ? homePoint.position : spawnPosition;
    }

    /// <summary>
    /// Builds the grid, runs A* from the robot to 'destination' and stores the result
    /// in 'corners'. Returns false (and logs why) if no route exists.
    /// </summary>
    private bool PlanPath(Vector3 destination)
    {
        corners = new Vector3[0];
        cornerIndex = 0;

        Vector3 start = robotBase.position;
        gridY = start.y;

        List<Bounds> obstacles = CollectObstacles();
        if (obstacles.Count == 0)
        {
            Debug.LogWarning("TurtleBotNavigator: no obstacles found, so the robot will drive in a straight " +
                             "line. Check 'Obstacle Tags' (shelves must have the tag 'Shelf').", this);
        }

        // Size the grid so it covers everything relevant.
        Bounds area = new Bounds(start, Vector3.zero);
        area.Encapsulate(destination);
        area.Encapsulate(GetHomePosition());
        if (target != null) area.Encapsulate(target.position);
        foreach (Bounds b in obstacles) area.Encapsulate(b);

        float pad = Mathf.Max(gridPadding, robotRadius + 2f * cellSize);
        area.Expand(pad * 2f); // Expand() grows the total size, so this adds 'pad' on each side

        gridOrigin = new Vector2(area.min.x, area.min.z);
        gridW = Mathf.CeilToInt(area.size.x / cellSize);
        gridH = Mathf.CeilToInt(area.size.z / cellSize);

        if (gridW <= 0 || gridH <= 0 || (long)gridW * gridH > MaxGridCells)
        {
            Debug.LogWarning("TurtleBotNavigator: grid would be " + gridW + " x " + gridH +
                             " cells. Increase 'Cell Size' or check the obstacle setup.", this);
            blocked = null;
            return false;
        }

        blocked = new bool[gridW * gridH];
        foreach (Bounds b in obstacles) MarkBlocked(b);

        int startCell = FindNearestFreeCell(start, null);
        if (startCell < 0)
        {
            Debug.LogWarning("TurtleBotNavigator: no free cell near the robot. Lower 'Robot Radius' " +
                             "or increase 'Snap Distance'.", this);
            return false;
        }

        bool[] reachable = FloodFill(startCell);
        int goalCell = FindNearestFreeCell(destination, reachable);
        if (goalCell < 0)
        {
            Debug.LogWarning("TurtleBotNavigator: no reachable free cell within " + snapDistance +
                             " m of the destination. Increase 'Snap Distance' or lower 'Robot Radius'.", this);
            return false;
        }

        List<int> cellPath = AStar(startCell, goalCell);
        if (cellPath == null)
        {
            Debug.LogWarning("TurtleBotNavigator: A* found no route to the destination.", this);
            return false;
        }

        List<Vector3> points = new List<Vector3>(cellPath.Count);
        foreach (int cell in cellPath) points.Add(CellCenter(cell));

        corners = Smooth(points).ToArray();
        cornerIndex = corners.Length > 1 ? 1 : 0; // corners[0] is roughly where we are now
        return true;
    }

    private List<Bounds> CollectObstacles()
    {
        List<Bounds> result = new List<Bounds>();
        HashSet<Renderer> seen = new HashSet<Renderer>();
        float maxBottom = robotBase.position.y + overheadClearance;

        if (obstacleTags != null)
        {
            foreach (string tag in obstacleTags)
            {
                if (string.IsNullOrEmpty(tag)) continue;

                GameObject[] objects;
                try
                {
                    objects = GameObject.FindGameObjectsWithTag(tag);
                }
                catch (UnityException)
                {
                    Debug.LogWarning("TurtleBotNavigator: the tag '" + tag + "' is not defined in this project.", this);
                    continue;
                }

                foreach (GameObject go in objects) AddRenderers(go.transform, result, seen, maxBottom);
            }
        }

        if (extraObstacleRoots != null)
        {
            foreach (Transform root in extraObstacleRoots)
            {
                if (root != null) AddRenderers(root, result, seen, maxBottom);
            }
        }

        return result;
    }

    private void AddRenderers(Transform root, List<Bounds> result, HashSet<Renderer> seen, float maxBottom)
    {
        foreach (Renderer r in root.GetComponentsInChildren<Renderer>())
        {
            if (!r.enabled || !seen.Add(r)) continue;
            if (r.transform.IsChildOf(transform)) continue; // never treat the robot itself as an obstacle

            Bounds b = r.bounds;
            if (b.min.y > maxBottom) continue; // hangs overhead, the robot can pass under
            result.Add(b);
        }
    }

    private void MarkBlocked(Bounds b)
    {
        // Inflate by the robot radius: the robot's turning centre must stay this far away.
        float minX = b.min.x - robotRadius;
        float maxX = b.max.x + robotRadius;
        float minZ = b.min.z - robotRadius;
        float maxZ = b.max.z + robotRadius;

        int ix0 = Mathf.Max(0, Mathf.FloorToInt((minX - gridOrigin.x) / cellSize));
        int ix1 = Mathf.Min(gridW - 1, Mathf.FloorToInt((maxX - gridOrigin.x) / cellSize));
        int iz0 = Mathf.Max(0, Mathf.FloorToInt((minZ - gridOrigin.y) / cellSize));
        int iz1 = Mathf.Min(gridH - 1, Mathf.FloorToInt((maxZ - gridOrigin.y) / cellSize));

        for (int iz = iz0; iz <= iz1; iz++)
        {
            for (int ix = ix0; ix <= ix1; ix++)
            {
                float cx = gridOrigin.x + (ix + 0.5f) * cellSize;
                float cz = gridOrigin.y + (iz + 0.5f) * cellSize;
                if (cx >= minX && cx <= maxX && cz >= minZ && cz <= maxZ)
                {
                    blocked[iz * gridW + ix] = true;
                }
            }
        }
    }

    // ---- grid helpers ----

    private int WorldToCell(Vector3 p)
    {
        int ix = Mathf.FloorToInt((p.x - gridOrigin.x) / cellSize);
        int iz = Mathf.FloorToInt((p.z - gridOrigin.y) / cellSize);
        if (ix < 0 || iz < 0 || ix >= gridW || iz >= gridH) return -1;
        return iz * gridW + ix;
    }

    private Vector3 CellCenter(int cell)
    {
        int ix = cell % gridW;
        int iz = cell / gridW;
        return new Vector3(
            gridOrigin.x + (ix + 0.5f) * cellSize,
            gridY,
            gridOrigin.y + (iz + 0.5f) * cellSize);
    }

    /// <summary>
    /// Nearest free cell (straight-line distance) to 'p', within 'snapDistance'.
    /// If 'mask' is given, only cells with mask[cell] == true are considered.
    /// </summary>
    private int FindNearestFreeCell(Vector3 p, bool[] mask)
    {
        int cx = Mathf.Clamp(Mathf.FloorToInt((p.x - gridOrigin.x) / cellSize), 0, gridW - 1);
        int cz = Mathf.Clamp(Mathf.FloorToInt((p.z - gridOrigin.y) / cellSize), 0, gridH - 1);
        int maxR = Mathf.CeilToInt(snapDistance / cellSize);

        int best = -1;
        float bestSqr = float.MaxValue;

        for (int dz = -maxR; dz <= maxR; dz++)
        {
            for (int dx = -maxR; dx <= maxR; dx++)
            {
                int ix = cx + dx;
                int iz = cz + dz;
                if (ix < 0 || iz < 0 || ix >= gridW || iz >= gridH) continue;

                int idx = iz * gridW + ix;
                if (blocked[idx]) continue;
                if (mask != null && !mask[idx]) continue;

                Vector3 c = CellCenter(idx);
                float ddx = c.x - p.x;
                float ddz = c.z - p.z;
                float sqr = ddx * ddx + ddz * ddz;
                if (sqr < bestSqr)
                {
                    bestSqr = sqr;
                    best = idx;
                }
            }
        }
        return best;
    }

    /// <summary>Marks every free cell that can be reached from 'start' (4-connected).</summary>
    private bool[] FloodFill(int start)
    {
        bool[] visited = new bool[gridW * gridH];
        Queue<int> queue = new Queue<int>();
        queue.Enqueue(start);
        visited[start] = true;

        while (queue.Count > 0)
        {
            int cell = queue.Dequeue();
            int cx = cell % gridW;
            int cz = cell / gridW;

            TryVisit(cx + 1, cz, visited, queue);
            TryVisit(cx - 1, cz, visited, queue);
            TryVisit(cx, cz + 1, visited, queue);
            TryVisit(cx, cz - 1, visited, queue);
        }
        return visited;
    }

    private void TryVisit(int ix, int iz, bool[] visited, Queue<int> queue)
    {
        if (ix < 0 || iz < 0 || ix >= gridW || iz >= gridH) return;
        int idx = iz * gridW + ix;
        if (visited[idx] || blocked[idx]) return;
        visited[idx] = true;
        queue.Enqueue(idx);
    }

    // ---- A* (binary-heap open list: grids at 0.1 m are large) ----

    private List<int> AStar(int start, int goal)
    {
        int n = gridW * gridH;
        float[] g = new float[n];
        int[] parent = new int[n];
        bool[] closed = new bool[n];
        for (int i = 0; i < n; i++)
        {
            g[i] = float.MaxValue;
            parent[i] = -1;
        }

        MinHeap open = new MinHeap();
        g[start] = 0f;
        open.Push(start, Heuristic(start, goal));

        while (open.Count > 0)
        {
            int current = open.Pop();
            if (closed[current]) continue; // stale heap entry
            if (current == goal) return Reconstruct(parent, current);

            closed[current] = true;
            int cx = current % gridW;
            int cz = current / gridW;

            for (int dz = -1; dz <= 1; dz++)
            {
                for (int dx = -1; dx <= 1; dx++)
                {
                    if (dx == 0 && dz == 0) continue;

                    int nx = cx + dx;
                    int nz = cz + dz;
                    if (nx < 0 || nz < 0 || nx >= gridW || nz >= gridH) continue;

                    int ni = nz * gridW + nx;
                    if (blocked[ni] || closed[ni]) continue;

                    bool diagonal = dx != 0 && dz != 0;
                    // No squeezing diagonally between two blocked cells.
                    if (diagonal && (blocked[cz * gridW + nx] || blocked[nz * gridW + cx])) continue;

                    float newG = g[current] + (diagonal ? 1.4142f : 1f);
                    if (newG < g[ni])
                    {
                        g[ni] = newG;
                        parent[ni] = current;
                        open.Push(ni, newG + Heuristic(ni, goal));
                    }
                }
            }
        }

        return null; // open list exhausted: no route
    }

    private float Heuristic(int a, int b)
    {
        // Octile distance: exact cost on an empty 8-connected grid.
        int dx = Mathf.Abs(a % gridW - b % gridW);
        int dz = Mathf.Abs(a / gridW - b / gridW);
        return (dx + dz) + (1.4142f - 2f) * Mathf.Min(dx, dz);
    }

    private static List<int> Reconstruct(int[] parent, int current)
    {
        List<int> path = new List<int>();
        while (current != -1)
        {
            path.Add(current);
            current = parent[current];
        }
        path.Reverse();
        return path;
    }

    /// <summary>Minimal binary min-heap of (cell, priority).</summary>
    private sealed class MinHeap
    {
        private readonly List<int> cells = new List<int>();
        private readonly List<float> keys = new List<float>();

        public int Count => cells.Count;

        public void Push(int cell, float key)
        {
            cells.Add(cell);
            keys.Add(key);
            int i = cells.Count - 1;
            while (i > 0)
            {
                int p = (i - 1) / 2;
                if (keys[p] <= keys[i]) break;
                Swap(i, p);
                i = p;
            }
        }

        public int Pop()
        {
            int top = cells[0];
            int last = cells.Count - 1;
            cells[0] = cells[last];
            keys[0] = keys[last];
            cells.RemoveAt(last);
            keys.RemoveAt(last);

            int i = 0;
            while (true)
            {
                int l = 2 * i + 1;
                int r = l + 1;
                int smallest = i;
                if (l < cells.Count && keys[l] < keys[smallest]) smallest = l;
                if (r < cells.Count && keys[r] < keys[smallest]) smallest = r;
                if (smallest == i) break;
                Swap(i, smallest);
                i = smallest;
            }
            return top;
        }

        private void Swap(int a, int b)
        {
            int c = cells[a]; cells[a] = cells[b]; cells[b] = c;
            float k = keys[a]; keys[a] = keys[b]; keys[b] = k;
        }
    }

    // ---- path smoothing ----

    /// <summary>
    /// Grid paths zig-zag in 45-degree steps. Keep only the points needed so that every
    /// straight segment between kept points stays inside free cells.
    /// </summary>
    private List<Vector3> Smooth(List<Vector3> points)
    {
        List<Vector3> result = new List<Vector3>();
        if (points.Count == 0) return result;

        int i = 0;
        result.Add(points[0]);

        while (i < points.Count - 1)
        {
            int j = points.Count - 1;
            while (j > i + 1 && !HasLineOfSight(points[i], points[j])) j--;
            result.Add(points[j]);
            i = j;
        }
        return result;
    }

    private bool HasLineOfSight(Vector3 a, Vector3 b)
    {
        float dist = FlatDistance(a, b);
        int steps = Mathf.Max(1, Mathf.CeilToInt(dist / (cellSize * 0.5f)));

        for (int s = 0; s <= steps; s++)
        {
            int cell = WorldToCell(Vector3.Lerp(a, b, s / (float)steps));
            if (cell < 0 || blocked[cell]) return false;
        }
        return true;
    }

    // ------------------------------------------------------------------
    // Layer 2: local follower -> velocity command
    // ------------------------------------------------------------------

    /// <summary>
    /// Sends one velocity command toward the next corner. Returns true (and stops the robot)
    /// once within 'stopDistance' of the final corner.
    /// </summary>
    private bool FollowPath(float stopDistance)
    {
        if (corners == null || corners.Length == 0)
        {
            drive.Stop();
            return true;
        }

        Vector3 pos = robotBase.position;
        int lastIndex = corners.Length - 1;
        float distToFinal = FlatDistance(pos, corners[lastIndex]);

        if (distToFinal <= stopDistance)
        {
            drive.Stop();
            return true;
        }

        while (cornerIndex < lastIndex && FlatDistance(pos, corners[cornerIndex]) <= waypointTolerance)
        {
            cornerIndex++;
        }

        Vector3 toCorner = corners[cornerIndex] - pos;
        toCorner.y = 0f;
        if (toCorner.sqrMagnitude < 0.0001f)
        {
            drive.Stop();
            return cornerIndex == lastIndex;
        }

        // Heading error. Unity's SignedAngle is positive for a clockwise turn seen from above;
        // ROS angular velocity is positive counter-clockwise, hence the minus sign.
        Vector3 forward = robotBase.forward;
        forward.y = 0f;
        float errorDeg = Vector3.SignedAngle(forward, toCorner, Vector3.up);
        float angular = -headingGain * errorDeg * Mathf.Deg2Rad;

        // Speed: full on a clear straight, less when turning or close to the goal,
        // zero beyond 'pivotAngle' so the robot turns on the spot instead of circling.
        float speedFactor = Mathf.Clamp01(1f - Mathf.Abs(errorDeg) / pivotAngle);
        if (cornerIndex == lastIndex && distToFinal < slowDownDistance)
        {
            speedFactor *= Mathf.Max(0.25f, distToFinal / slowDownDistance);
        }

        drive.SetCommand(cruiseSpeed * speedFactor, angular);
        return false;
    }

    private static float FlatDistance(Vector3 a, Vector3 b)
    {
        a.y = 0f;
        b.y = 0f;
        return Vector3.Distance(a, b);
    }

    // ------------------------------------------------------------------
    // Gizmos
    // ------------------------------------------------------------------

    void OnDrawGizmos()
    {
        if (drawGrid && blocked != null)
        {
            Gizmos.color = new Color(1f, 0f, 0f, 0.3f);
            Vector3 size = new Vector3(cellSize * 0.95f, 0.02f, cellSize * 0.95f);
            for (int i = 0; i < blocked.Length; i++)
            {
                if (blocked[i]) Gizmos.DrawCube(CellCenter(i), size);
            }
        }

        if (drawPath && corners != null && corners.Length > 0)
        {
            Vector3 lift = Vector3.up * 0.05f;
            Gizmos.color = Color.yellow;
            for (int i = 0; i < corners.Length; i++)
            {
                Gizmos.DrawSphere(corners[i] + lift, 0.06f);
                if (i < corners.Length - 1) Gizmos.DrawLine(corners[i] + lift, corners[i + 1] + lift);
            }
        }
    }

    // Arrival radii when selected (red = shelf, green = home).
    void OnDrawGizmosSelected()
    {
        if (target != null)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(target.position, stoppingDistance);
        }

        Vector3 home = homePoint != null ? homePoint.position : (robotBase != null ? robotBase.position : transform.position);
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(home, homeStoppingDistance);
    }
}
