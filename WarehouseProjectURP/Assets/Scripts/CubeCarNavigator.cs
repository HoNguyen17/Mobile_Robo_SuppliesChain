using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Simplified "drives like a car" navigator for a placeholder cube.
/// No WheelColliders/Rigidbody physics and NO NavMesh bake needed - just Transform
/// manipulation plus a small self-made path planner.
///
/// Two layers:
///   1. GLOBAL PLANNER  - builds an occupancy grid of the floor (cells covered by shelves,
///                        inflated by the robot's radius, are "blocked"), then runs A* to
///                        find the shortest free route and straightens it into a few corners.
///   2. LOCAL FOLLOWER  - drives along those corners: only moves forward along its own
///                        facing direction, turns while driving, pivots on the spot when
///                        it has to turn sharply, and slows down near the final goal.
///
/// Mission flow:
///   Idle -> (GoToTarget) -> DrivingToTarget -> WaitingAtTarget -> DrivingHome -> Idle
///
/// Obstacles = every Renderer under GameObjects that have one of the 'Obstacle Tags'
/// (default: "Shelf"), plus any roots you drag into 'Extra Obstacle Roots'.
/// The grid is rebuilt every time a path is planned, so regenerating the warehouse is fine.
/// </summary>
public class CubeCarNavigator : MonoBehaviour
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

    [Tooltip("Where the cube returns to. Leave empty to return to the spawn position. " +
             "IMPORTANT: do NOT make this a child of the cube, or it will move with it.")]
    public Transform homePoint;

    [Header("Mission Behaviour")]
    [Tooltip("Start driving to the target automatically when Play begins.")]
    public bool autoStart = true;

    [Tooltip("After reaching the target, wait and then drive home automatically.")]
    public bool autoReturnHome = true;

    [Tooltip("Seconds to wait at the shelf before returning (simulated pick-up time).")]
    public float waitTimeAtTarget = 2f;

    [Header("Movement Settings")]
    [Tooltip("How fast the cube drives forward, in units per second.")]
    public float moveSpeed = 5f;

    [Tooltip("How fast the cube turns, in degrees per second.")]
    public float turnSpeed = 90f;

    [Tooltip("Stop once within this distance of the free spot in front of the target shelf. Suggested: 0.3")]
    public float stoppingDistance = 0.3f;

    [Tooltip("Stop once within this distance of the home point.")]
    public float homeStoppingDistance = 0.5f;

    [Tooltip("Start slowing down once within this distance of the final goal.")]
    public float slowDownDistance = 4f;

    [Tooltip("A path corner counts as 'reached' within this distance. " +
             "Bigger = smoother but cuts corners closer to the shelves.")]
    public float waypointTolerance = 0.3f;

    [Tooltip("If the next corner is more than this many degrees away from where the cube is facing, " +
             "the cube stops and turns on the spot (like a differential-drive robot) before driving on. " +
             "Below this angle it drives, slower the more it still has to turn. " +
             "This is what stops it from circling around a corner it can't turn toward.")]
    [Range(10f, 170f)]
    public float pivotAngle = 45f;

    [Header("Path Planning (Occupancy Grid + A*)")]
    [Tooltip("Every Renderer under a GameObject with one of these tags is an obstacle.")]
    public string[] obstacleTags = { "Shelf" };

    [Tooltip("Optional: extra parent objects whose child Renderers are also obstacles (e.g. Walls).")]
    public Transform[] extraObstacleRoots;

    [Tooltip("Size of one grid cell in world units. Smaller = more precise but slower to plan.")]
    public float cellSize = 0.5f;

    [Tooltip("Safety margin around obstacles. The cube's half-diagonal is ~0.71, so keep this above that. " +
             "If the cube clips a shelf corner, raise it; if 'no path' appears, lower it.")]
    public float robotRadius = 1f;

    [Tooltip("Renderers that start higher than this above the cube's center (e.g. ceiling lights) are ignored.")]
    public float overheadClearance = 1.5f;

    [Tooltip("Extra free space around everything, so paths can go around the outermost shelves.")]
    public float gridPadding = 3f;

    [Tooltip("How far to search for a free cell when the cube/target/home sits inside a blocked area " +
             "(a shelf's center is always blocked).")]
    public float snapDistance = 6f;

    [Header("Debug")]
    [Tooltip("T = go to target, H = return home. Uses the old Input Manager.")]
    public bool enableDebugKeys = true;

    [Tooltip("Draw the blocked cells (red) in the Scene view / Game view with Gizmos on.")]
    public bool drawGrid = true;

    /// <summary>Current state, readable from other scripts.</summary>
    public NavState State { get; private set; } = NavState.Idle;

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
        // Remember where we started so "home" works even without a homePoint Transform.
        spawnPosition = transform.position;
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
    // Public API - call these from other scripts, UI buttons, or the debug keys
    // ------------------------------------------------------------------

    /// <summary>Plan a path to the target shelf and start driving.</summary>
    public void GoToTarget()
    {
        if (target == null)
        {
            Debug.LogWarning("CubeCarNavigator: no target assigned.", this);
            return;
        }

        if (!PlanPath(target.position))
        {
            State = NavState.Idle;
            return;
        }
        State = NavState.DrivingToTarget;
    }

    /// <summary>Plan a path back to the home point and start driving.</summary>
    public void ReturnHome()
    {
        if (!PlanPath(GetHomePosition()))
        {
            State = NavState.Idle;
            return;
        }
        State = NavState.DrivingHome;
    }

    /// <summary>Stop where we are.</summary>
    public void Stop()
    {
        State = NavState.Idle;
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
        }

        switch (State)
        {
            case NavState.DrivingToTarget:
                if (FollowPath(stoppingDistance))
                {
                    // Arrived at the shelf.
                    waitTimer = waitTimeAtTarget;
                    State = autoReturnHome ? NavState.WaitingAtTarget : NavState.Idle;
                }
                break;

            case NavState.WaitingAtTarget:
                waitTimer -= Time.deltaTime;
                if (waitTimer <= 0f)
                {
                    ReturnHome();
                }
                break;

            case NavState.DrivingHome:
                if (FollowPath(homeStoppingDistance))
                {
                    // Back home, mission complete.
                    State = NavState.Idle;
                }
                break;

            case NavState.Idle:
            default:
                break;
        }
    }

    // ------------------------------------------------------------------
    // Layer 1: global planning (occupancy grid + A*)
    // ------------------------------------------------------------------

    private Vector3 GetHomePosition()
    {
        return homePoint != null ? homePoint.position : spawnPosition;
    }

    // ---- Read-only access to the occupancy grid (used by the ROS map publisher) ----

    /// <summary>Grid size in cells along Unity X.</summary>
    public int GridWidth { get { return gridW; } }

    /// <summary>Grid size in cells along Unity Z.</summary>
    public int GridHeight { get { return gridH; } }

    /// <summary>World (x, z) of the grid's minimum corner.</summary>
    public Vector2 GridOriginXZ { get { return gridOrigin; } }

    /// <summary>Blocked flags, index = iz * GridWidth + ix. Already inflated by Robot Radius.</summary>
    public bool[] BlockedCells { get { return blocked; } }

    /// <summary>
    /// Builds the occupancy grid around the cube's current position WITHOUT planning a path.
    /// Works even while this component is unticked. Returns false if the grid could not be built.
    /// </summary>
    public bool BuildGridForRos()
    {
        return BuildGrid(transform.position, transform.position);
    }

    /// <summary>
    /// Steps 1-3 of planning: find obstacles, size the grid, mark blocked cells.
    /// Returns false (and logs why) if the grid could not be built.
    /// </summary>
    private bool BuildGrid(Vector3 start, Vector3 destination)
    {
        gridY = start.y;

        // 1. Find obstacles.
        List<Bounds> obstacles = CollectObstacles();
        if (obstacles.Count == 0)
        {
            Debug.LogWarning("CubeCarNavigator: no obstacles found, so the cube will drive in a straight " +
                             "line. Check 'Obstacle Tags' (shelves must have the tag 'Shelf') or add " +
                             "parents to 'Extra Obstacle Roots'.", this);
        }

        // 2. Size the grid so it covers everything relevant.
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

        if (gridW <= 0 || gridH <= 0 || (long)gridW * gridH > 1000000L)
        {
            Debug.LogWarning("CubeCarNavigator: grid would be " + gridW + " x " + gridH +
                             " cells. Increase 'Cell Size' or check the obstacle setup.", this);
            blocked = null;
            return false;
        }

        // 3. Mark blocked cells (obstacles inflated by the robot radius).
        blocked = new bool[gridW * gridH];
        foreach (Bounds b in obstacles) MarkBlocked(b);
        return true;
    }

    /// <summary>
    /// Builds the grid, runs A* from the cube to 'destination' and stores the result
    /// in 'corners'. Returns false (and logs why) if no route exists.
    /// </summary>
    private bool PlanPath(Vector3 destination)
    {
        corners = new Vector3[0];
        cornerIndex = 0;

        Vector3 start = transform.position;

        // Steps 1-3: obstacles -> grid -> blocked cells.
        if (!BuildGrid(start, destination)) return false;

        // 4. Start cell = nearest free cell to the cube.
        int startCell = FindNearestFreeCell(start, null);
        if (startCell < 0)
        {
            Debug.LogWarning("CubeCarNavigator: no free cell near the cube. Lower 'Robot Radius' " +
                             "or increase 'Snap Distance'.", this);
            return false;
        }

        // 5. Goal cell = nearest free cell to the destination that the cube can actually reach.
        bool[] reachable = FloodFill(startCell);
        int goalCell = FindNearestFreeCell(destination, reachable);
        if (goalCell < 0)
        {
            Debug.LogWarning("CubeCarNavigator: no reachable free cell within " + snapDistance +
                             " units of the destination. Increase 'Snap Distance' or lower 'Robot Radius'.", this);
            return false;
        }

        // 6. A*.
        List<int> cellPath = AStar(startCell, goalCell);
        if (cellPath == null)
        {
            Debug.LogWarning("CubeCarNavigator: A* found no route to the destination.", this);
            return false;
        }

        // 7. Turn cells into world points and straighten the zig-zag.
        List<Vector3> points = new List<Vector3>();
        for (int i = 0; i < cellPath.Count; i++)
        {
            points.Add(CellCenter(cellPath[i]));
        }

        corners = Smooth(points).ToArray();
        // corners[0] is (roughly) where we are right now, so start with the next one.
        cornerIndex = corners.Length > 1 ? 1 : 0;
        return true;
    }

    private List<Bounds> CollectObstacles()
    {
        List<Bounds> result = new List<Bounds>();
        HashSet<Renderer> seen = new HashSet<Renderer>();
        float maxBottom = transform.position.y + overheadClearance;

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
                    Debug.LogWarning("CubeCarNavigator: the tag '" + tag + "' is not defined in this project.", this);
                    continue;
                }

                foreach (GameObject go in objects)
                {
                    AddRenderers(go.transform, result, seen, maxBottom);
                }
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
            if (r.transform.IsChildOf(transform)) continue; // never treat the cube itself as an obstacle

            Bounds b = r.bounds;
            if (b.min.y > maxBottom) continue; // hangs overhead (ceiling lights etc.), cube can pass under
            result.Add(b);
        }
    }

    private void MarkBlocked(Bounds b)
    {
        // Inflate by the robot radius: the cube's CENTER must stay this far from the obstacle.
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
        List<int> queue = new List<int>();
        queue.Add(start);
        visited[start] = true;

        for (int head = 0; head < queue.Count; head++)
        {
            int cell = queue[head];
            int cx = cell % gridW;
            int cz = cell / gridW;

            TryVisit(cx + 1, cz, visited, queue);
            TryVisit(cx - 1, cz, visited, queue);
            TryVisit(cx, cz + 1, visited, queue);
            TryVisit(cx, cz - 1, visited, queue);
        }
        return visited;
    }

    private void TryVisit(int ix, int iz, bool[] visited, List<int> queue)
    {
        if (ix < 0 || iz < 0 || ix >= gridW || iz >= gridH) return;
        int idx = iz * gridW + ix;
        if (visited[idx] || blocked[idx]) return;
        visited[idx] = true;
        queue.Add(idx);
    }

    // ---- A* ----

    private List<int> AStar(int start, int goal)
    {
        int n = gridW * gridH;
        float[] g = new float[n];
        int[] parent = new int[n];
        bool[] closed = new bool[n];
        bool[] inOpen = new bool[n];
        for (int i = 0; i < n; i++)
        {
            g[i] = float.MaxValue;
            parent[i] = -1;
        }

        List<int> open = new List<int>();
        g[start] = 0f;
        open.Add(start);
        inOpen[start] = true;

        while (open.Count > 0)
        {
            // Pick the open cell with the lowest f = g + heuristic.
            int bestK = 0;
            float bestF = float.MaxValue;
            for (int k = 0; k < open.Count; k++)
            {
                int c = open[k];
                float f = g[c] + Heuristic(c, goal);
                if (f < bestF)
                {
                    bestF = f;
                    bestK = k;
                }
            }

            int current = open[bestK];
            open[bestK] = open[open.Count - 1];
            open.RemoveAt(open.Count - 1);
            inOpen[current] = false;

            if (current == goal)
            {
                return Reconstruct(parent, current);
            }

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
                        if (!inOpen[ni])
                        {
                            open.Add(ni);
                            inOpen[ni] = true;
                        }
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

    private List<int> Reconstruct(int[] parent, int current)
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
            while (j > i + 1 && !HasLineOfSight(points[i], points[j]))
            {
                j--;
            }
            result.Add(points[j]);
            i = j;
        }
        return result;
    }

    private bool HasLineOfSight(Vector3 a, Vector3 b)
    {
        float dx = b.x - a.x;
        float dz = b.z - a.z;
        float dist = Mathf.Sqrt(dx * dx + dz * dz);
        int steps = Mathf.Max(1, Mathf.CeilToInt(dist / (cellSize * 0.5f)));

        for (int s = 0; s <= steps; s++)
        {
            Vector3 p = Vector3.Lerp(a, b, s / (float)steps);
            int cell = WorldToCell(p);
            if (cell < 0 || blocked[cell]) return false;
        }
        return true;
    }

    // ------------------------------------------------------------------
    // Layer 2: local car-like follower
    // ------------------------------------------------------------------

    /// <summary>
    /// Drives along the planned corners. Returns true once the cube is within
    /// 'stopDistance' of the final corner.
    /// </summary>
    private bool FollowPath(float stopDistance)
    {
        if (corners == null || corners.Length == 0)
        {
            return true; // nothing to follow
        }

        int lastIndex = corners.Length - 1;
        float distToFinal = FlatDistance(transform.position, corners[lastIndex]);

        if (distToFinal <= stopDistance)
        {
            return true; // arrived
        }

        // Advance to the next corner once we're close enough to the current one.
        while (cornerIndex < lastIndex &&
               FlatDistance(transform.position, corners[cornerIndex]) <= waypointTolerance)
        {
            cornerIndex++;
        }

        Vector3 toWaypoint = corners[cornerIndex] - transform.position;
        toWaypoint.y = 0f; // stay on the ground plane, ignore height differences

        if (toWaypoint.sqrMagnitude < 0.0001f)
        {
            return cornerIndex == lastIndex;
        }

        Vector3 dir = toWaypoint.normalized;

        // --- Turning: rotate gradually toward the next corner ---
        Quaternion targetRotation = Quaternion.LookRotation(dir, Vector3.up);
        transform.rotation = Quaternion.RotateTowards(
            transform.rotation,
            targetRotation,
            turnSpeed * Time.deltaTime
        );

        // --- Speed: 1.0 on a clear straight, less when turning or close to the goal ---
        float speedFactor = 1f;

        // Slow down on the final approach (same idea as the original "throttle" logic).
        if (cornerIndex == lastIndex && distToFinal < slowDownDistance)
        {
            speedFactor = distToFinal / slowDownDistance;
        }

        // Only drive once we're roughly facing the next corner. Beyond 'pivotAngle' the
        // speed is zero, so the cube pivots on the spot instead of driving in a circle
        // around a corner that sits inside its turning radius.
        float angleToCorner = Vector3.Angle(transform.forward, dir);
        float alignment = Mathf.Clamp01(1f - angleToCorner / pivotAngle);
        speedFactor *= alignment;

        // --- Moving: always drive forward along however we're currently facing ---
        transform.position += transform.forward * moveSpeed * speedFactor * Time.deltaTime;

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
        // Blocked cells (red): this is what the planner "sees" as shelves + safety margin.
        if (drawGrid && blocked != null)
        {
            Gizmos.color = new Color(1f, 0f, 0f, 0.3f);
            Vector3 size = new Vector3(cellSize * 0.95f, 0.05f, cellSize * 0.95f);
            for (int i = 0; i < blocked.Length; i++)
            {
                if (blocked[i]) Gizmos.DrawCube(CellCenter(i), size);
            }
        }

        // Planned path (yellow).
        if (corners != null && corners.Length > 0)
        {
            Vector3 lift = Vector3.up * 0.1f;
            Gizmos.color = Color.yellow;
            for (int i = 0; i < corners.Length; i++)
            {
                Gizmos.DrawSphere(corners[i] + lift, 0.15f);
                if (i < corners.Length - 1)
                {
                    Gizmos.DrawLine(corners[i] + lift, corners[i + 1] + lift);
                }
            }
        }
    }

    // Draw the arrival radii when the cube is selected (red = shelf, green = home).
    void OnDrawGizmosSelected()
    {
        if (target != null)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(target.position, stoppingDistance);
        }

        Vector3 home = homePoint != null
            ? homePoint.position
            : (Application.isPlaying ? spawnPosition : transform.position);
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(home, homeStoppingDistance);
    }
}