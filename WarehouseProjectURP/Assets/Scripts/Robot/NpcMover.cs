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

    void Start()
    {
        transform.position = waypointA;
    }

    void Update()
    {
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
