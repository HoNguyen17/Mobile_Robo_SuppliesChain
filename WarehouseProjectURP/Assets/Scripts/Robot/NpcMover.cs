using UnityEngine;

/// <summary>
/// Moves an NPC worker across the aisle on scripted waypoints for dynamic obstacle avoidance (S-04).
/// 
/// Conforms to docs/architecture.md §2 and docs/test-plan.md §1 (S-04: NPC walks across path at 0.5 m/s, every 8 s).
/// </summary>
[DisallowMultipleComponent]
public class NpcMover : MonoBehaviour
{
    [Tooltip("Walking speed in robot m/s (default 0.5 m/s).")]
    public float speedRobotMps = 0.5f;

    [Tooltip("World scale factor (default 4.0).")]
    public float worldScale = 4.0f;

    [Tooltip("Start waypoint in Unity coordinates.")]
    public Vector3 waypointA = new Vector3(-8f, 1f, 0f);

    [Tooltip("End waypoint in Unity coordinates.")]
    public Vector3 waypointB = new Vector3(8f, 1f, 0f);

    [Tooltip("Time period for crossing cycle in seconds.")]
    public float cyclePeriodSeconds = 8.0f;

    private float timer = 0f;
    private bool movingToB = true;

    // Spec-driven mode (ADR-019): the position is NpcPath.PositionAt(elapsed). Null = the old two-waypoint behaviour.
    private NpcPath specPath;
    private float specElapsed;
    private float specUnityHeight = 1f;

    /// <summary>The worker's position in ROS metres, or null for a legacy worker (which has no spec).</summary>
    public Vector2? RosPosition { get { return specPath == null ? (Vector2?)null : specPath.PositionAt(specElapsed); } }

    /// <summary>Heading in ROS radians (0 = +x, counter-clockwise); keeps the last heading while the worker stands still.</summary>
    public float RosYaw { get; private set; }

    /// <summary>
    /// Switches this worker to spec-driven mode. Call it before the object is first activated; the worker then stands at
    /// the first waypoint and moves along the path from the moment it is activated.
    /// </summary>
    public void InitFromSpec(NpcPath path, float scale, float unityHeight)
    {
        specPath = path;
        worldScale = scale;
        specUnityHeight = unityHeight;
        specElapsed = 0f;
        ApplySpecPose();
    }

    private void ApplySpecPose()
    {
        Vector2 p = specPath.PositionAt(specElapsed);
        // ROS x = Z / scale, ROS y = -X / scale (RosConversions.ToMapPose)
        transform.position = new Vector3(-p.y * worldScale, specUnityHeight, p.x * worldScale);
        Vector2 h = specPath.HeadingAt(specElapsed);
        if (h.sqrMagnitude > 0.001f)
        {
            RosYaw = Mathf.Atan2(h.y, h.x);
            transform.rotation = Quaternion.LookRotation(new Vector3(-h.y, 0f, h.x));
        }
    }

    void Start()
    {
        if (specPath != null) return;
        transform.position = waypointA;
    }

    void Update()
    {
        if (specPath != null)
        {
            specElapsed += Time.deltaTime;
            ApplySpecPose();
            return;
        }

        timer += Time.deltaTime;
        float actualSpeed = speedRobotMps * worldScale; // 2.0 Unity units/s

        Vector3 target = movingToB ? waypointB : waypointA;
        transform.position = Vector3.MoveTowards(transform.position, target, actualSpeed * Time.deltaTime);

        // Turn towards direction
        Vector3 dir = (target - transform.position).normalized;
        if (dir.sqrMagnitude > 0.001f)
        {
            transform.rotation = Quaternion.LookRotation(dir);
        }

        // Switch direction every cyclePeriodSeconds or when arrived
        if (Vector3.Distance(transform.position, target) < 0.05f || timer >= cyclePeriodSeconds)
        {
            movingToB = !movingToB;
            timer = 0f;
        }
    }
}
