using System;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Resets the robot pose and scenario obstacles in response to ROS /sim/reset_scenario,
/// and allows interactive manual testing:
/// - Press 'O' or '0' (or 'B') in Unity Game view to drop/toggle an obstacle directly in front of the robot!
/// - Pre-places a non-static, bright red draggable obstacle in the main aisle that the user can freely move with mouse.
/// </summary>
[DisallowMultipleComponent]
public class ScenarioLoader : MonoBehaviour
{
    public string resetTopic = "/sim/reset_scenario";
    public string ackTopic = "/sim/ack";

    [Tooltip("Default start position in Unity coordinates. Converts from ROS (5.01, -0.05, 0).")]
    public Vector3 defaultStartUnityPos = new Vector3(0.20f, 0.08f, 20.04f);
    public Vector3 defaultStartUnityEuler = Vector3.zero;

    private ROSConnection ros;
    private DiffDriveController drive;
    private GameObject scenarioObstacleContainer;
    private GameObject manualDynamicObstacle;
    private GameObject prePlacedDraggableObstacle;

    void Start()
    {
        drive = GetComponent<DiffDriveController>();
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<StringMsg>(resetTopic, OnResetScenario);

        // Pre-place a non-static, draggable obstacle in the main warehouse aisle for easy mouse-dragging
        SpawnDraggableAisleObstacle();
    }

    void Update()
    {
        // Support both letter 'O' and number '0', as well as 'B' (Box)
        if (Input.GetKeyDown(KeyCode.O) || Input.GetKeyDown(KeyCode.Alpha0) || 
            Input.GetKeyDown(KeyCode.Keypad0) || Input.GetKeyDown(KeyCode.B))
        {
            ToggleObstacleInFrontOfRobot();
        }

        // Keep draggable obstacle firmly on the floor so dragging in Scene view never lifts it above the LiDAR plane
        if (prePlacedDraggableObstacle != null)
        {
            Vector3 pos = prePlacedDraggableObstacle.transform.position;
            if (Mathf.Abs(pos.y - 1.5f) > 0.1f)
            {
                pos.y = 1.5f;
                prePlacedDraggableObstacle.transform.position = pos;
            }
        }
    }

    void OnDestroy()
    {
        if (ros != null)
        {
            ros.Unsubscribe(resetTopic);
        }
    }

    private static void SetTagSafely(GameObject go, string tagName)
    {
        try 
        { 
            go.tag = tagName; 
        } 
        catch 
        { 
            try { go.tag = "FloorBoxes"; } catch {} 
        }
    }

    /// <summary>
    /// Spawns a dedicated non-static, bright red box in the warehouse corridor that the user
    /// can select and move around with Unity's Move tool (W) without it being locked by static batching!
    /// </summary>
    private void SpawnDraggableAisleObstacle()
    {
        prePlacedDraggableObstacle = GameObject.Find("Dynamic_Draggable_Obstacle");
        if (prePlacedDraggableObstacle == null)
        {
            prePlacedDraggableObstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
            prePlacedDraggableObstacle.name = "Dynamic_Draggable_Obstacle";
        }

        SetTagSafely(prePlacedDraggableObstacle, "Obstacle");
        prePlacedDraggableObstacle.isStatic = false; // NON-STATIC! Can be freely dragged by mouse!

        // Position: height Y=1.5 with scale Y=3.0 so it reaches from floor (Y=0) up to 3m height!
        // LiDAR at Y=0.49m will NEVER miss it!
        prePlacedDraggableObstacle.transform.position = new Vector3(2.5f, 1.5f, 10.0f);
        prePlacedDraggableObstacle.transform.localScale = new Vector3(1.3f, 3.0f, 1.3f);

        var rb = prePlacedDraggableObstacle.GetComponent<Rigidbody>();
        if (rb == null) rb = prePlacedDraggableObstacle.AddComponent<Rigidbody>();
        rb.isKinematic = true;
        rb.useGravity = false;

        // Apply distinct hazard striped / bright red material
        Renderer rend = prePlacedDraggableObstacle.GetComponent<Renderer>();
        if (rend != null)
        {
            Shader shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
            if (shader != null)
            {
                Material mat = new Material(shader);
                mat.color = new Color(0.95f, 0.15f, 0.15f); // Bright Red Hazard
                rend.material = mat;
            }
        }

        Debug.Log("ScenarioLoader: 'Dynamic_Draggable_Obstacle' ready in aisle. Press 'O' or '0' to drop an obstacle, or drag this red cube in Scene view!");
    }

    /// <summary>
    /// Hotkey 'O' or '0': Toggles an obstacle 2.5m directly ahead of the robot.
    /// </summary>
    public void ToggleObstacleInFrontOfRobot()
    {
        if (manualDynamicObstacle != null)
        {
            Destroy(manualDynamicObstacle);
            manualDynamicObstacle = null;
            Debug.Log("[OBSTACLE CLEARED] Dynamic obstacle removed from path. Robot path will clear!");
            return;
        }

        Transform body = (drive != null && drive.BaseBody != null) ? drive.BaseBody : transform;
        Vector3 spawnPos = body.position + body.forward * 2.5f;
        spawnPos.y = 1.5f; // Grounded: extends from 0 to 3m height

        manualDynamicObstacle = CreateObstacleBox(spawnPos, new Vector3(1.3f, 3.0f, 1.3f), "Dynamic_Obstacle_Box");
        manualDynamicObstacle.isStatic = false;

        Renderer rend = manualDynamicObstacle.GetComponent<Renderer>();
        if (rend != null)
        {
            Shader shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
            if (shader != null)
            {
                Material mat = new Material(shader);
                mat.color = new Color(1.0f, 0.1f, 0.1f); // Bright Red Hazard
                rend.material = mat;
            }
        }

        Debug.Log($"[OBSTACLE SPAWNED] Dynamic obstacle placed 2.5m ahead of robot at {spawnPos}! Robot LiDAR will detect and replan. (Press 'O' or '0' again to remove)");
    }

    private void OnResetScenario(StringMsg msg)
    {
        string scenarioId = msg.data.Trim().ToUpper();
        try
        {
            ResetRobotPose();
            SetupScenarioObstacles(scenarioId);
            SendAck("reset_scenario", true, "");
            Debug.Log($"ScenarioLoader: Reset scenario {scenarioId} successfully.");
        }
        catch (Exception ex)
        {
            SendAck("reset_scenario", false, ex.Message);
            Debug.LogError($"ScenarioLoader reset error: {ex}");
        }
    }

    private void ResetRobotPose()
    {
        if (drive != null)
        {
            drive.SetCommand(0f, 0f);
        }

        transform.position = defaultStartUnityPos;
        transform.rotation = Quaternion.Euler(defaultStartUnityEuler);

        var ab = GetComponent<ArticulationBody>();
        if (ab != null)
        {
            ab.velocity = Vector3.zero;
            ab.angularVelocity = Vector3.zero;
            ab.TeleportRoot(defaultStartUnityPos, Quaternion.Euler(defaultStartUnityEuler));
        }
    }

    private void SetupScenarioObstacles(string scenarioId)
    {
        if (scenarioObstacleContainer != null)
        {
            Destroy(scenarioObstacleContainer);
        }

        scenarioObstacleContainer = new GameObject($"ScenarioObstacles_{scenarioId}");

        if (scenarioId == "S-01")
        {
            // S-01: Single 0.5m box on path (ROS: x=0.0, y=0.0 -> Unity: X=0, Z=0)
            CreateObstacleBox(new Vector3(0f, 1f, 0f), new Vector3(2f, 2f, 2f), "S01_Box");
        }
        else if (scenarioId == "S-02")
        {
            // S-02: 1.0m corridor (Unity: 4m wide) with boxes along sides
            for (int i = -3; i <= 3; i++)
            {
                float z = i * 4f;
                CreateObstacleBox(new Vector3(-2.5f, 1f, z), new Vector3(1.5f, 2f, 3f), $"S02_Left_{i}");
                CreateObstacleBox(new Vector3(2.5f, 1f, z), new Vector3(1.5f, 2f, 3f), $"S02_Right_{i}");
            }
        }
        else if (scenarioId == "S-03")
        {
            // S-03: Dead end - row of boxes closing the primary route
            for (int i = -2; i <= 2; i++)
            {
                CreateObstacleBox(new Vector3(i * 2f, 1f, 4f), new Vector3(2f, 2f, 2f), $"S03_Wall_{i}");
            }
        }
        else if (scenarioId == "S-04")
        {
            // S-04: Crossing NPC worker moving across path at 0.5 m/s
            CreateCrossingNpc(new Vector3(-6f, 1f, 0f), new Vector3(6f, 1f, 0f), 0.5f, "S04_Worker");
        }
    }

    private GameObject CreateCrossingNpc(Vector3 waypointA, Vector3 waypointB, float speed, string name)
    {
        GameObject npc = GameObject.CreatePrimitive(PrimitiveType.Capsule);
        npc.name = name;
        SetTagSafely(npc, "NPC");
        npc.isStatic = false;
        npc.transform.SetParent(scenarioObstacleContainer.transform);
        npc.transform.localScale = new Vector3(1.2f, 2.0f, 1.2f);

        var rb = npc.AddComponent<Rigidbody>();
        rb.isKinematic = true;
        rb.useGravity = false;

        var mover = npc.AddComponent<NpcMover>();
        mover.waypointA = waypointA;
        mover.waypointB = waypointB;
        mover.speedRobotMps = speed;
        mover.worldScale = 4.0f;

        return npc;
    }

    private GameObject CreateObstacleBox(Vector3 localPos, Vector3 size, string name)
    {
        GameObject box = GameObject.CreatePrimitive(PrimitiveType.Cube);
        box.name = name;
        SetTagSafely(box, "Obstacle");
        box.isStatic = false;
        if (scenarioObstacleContainer != null)
        {
            box.transform.SetParent(scenarioObstacleContainer.transform);
        }
        box.transform.position = localPos;
        box.transform.localScale = size;

        var rb = box.AddComponent<Rigidbody>();
        rb.isKinematic = true;
        rb.useGravity = false;

        return box;
    }

    private void SendAck(string cmd, bool ok, string error)
    {
        string json = $"{{\"cmd\":\"{cmd}\",\"ok\":{(ok ? "true" : "false")},\"error\":\"{error}\"}}";
        try
        {
            ros.Publish(ackTopic, new StringMsg(json));
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"SendAck error: {ex.Message}");
        }
    }
}
