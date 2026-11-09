using System;
using System.Collections.Generic;
using System.IO;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Resets the robot pose and scenario obstacles in response to ROS /sim/reset_scenario,
/// and allows interactive manual testing:
/// - Press 'O' or '0' (or 'B') in Unity Game view to drop/toggle an obstacle directly in front of the robot!
/// - Pre-places a non-static, bright red draggable obstacle in the main aisle that the user can freely move with mouse.
///
/// Two reset protocols (ADR-019). "S-04": the scenarios built in this file, as always. "S-06?seed=7": a scenario from
/// StreamingAssets/scenarios.json, with the draggable and manual boxes switched off and the workers started by the
/// robot leaving its start pose plus a seeded offset. The poses of the active workers go out on /sim/npc_poses
/// (ground truth for the metrics only, never for navigation).
/// </summary>
[DisallowMultipleComponent]
public class ScenarioLoader : MonoBehaviour
{
    public string resetTopic = "/sim/reset_scenario";
    public string ackTopic = "/sim/ack";

    [Tooltip("Default start position in Unity coordinates. Converts from ROS (5.01, -0.05, 0).")]
    public Vector3 defaultStartUnityPos = new Vector3(0.20f, 0.08f, 20.04f);
    public Vector3 defaultStartUnityEuler = Vector3.zero;

    [Header("Seeded scenarios (ADR-019)")]
    [Tooltip("File name in Assets/StreamingAssets, written by python3 -m warehouse_eval.scenario_spec --write.")]
    public string specFileName = "scenarios.json";
    [Tooltip("On a seeded reset, switch off the draggable box and the manual 'O' box so the scenario is the only dynamic object.")]
    public bool isolateSeededScenarios = true;
    public string npcPosesTopic = "/sim/npc_poses";
    public float npcPosesRateHz = 20f;
    [Tooltip("The workers of a robot_departs scenario start once the robot is this far (robot metres) from its start pose.")]
    public float departureDistanceM = 0.3f;
    [Tooltip("The robot is teleported on a reset; ignore its position for this long so the old position is not read as a departure.")]
    public float departureArmDelayS = 1.0f;

    private ROSConnection ros;
    private DiffDriveController drive;
    private GameObject scenarioObstacleContainer;
    private GameObject manualDynamicObstacle;
    private GameObject prePlacedDraggableObstacle;

    private class SpecWorker
    {
        public NpcWorkerSpec Spec;
        public NpcMover Mover;
        public bool Started;
        public double StartAtS = -1.0;   // set when the trigger fires
        public double DelayS;
    }

    private readonly List<SpecWorker> specWorkers = new List<SpecWorker>();
    private bool seededActive;
    private bool draggableHiddenBySeeded;
    private bool departed;
    private double resetAtS;
    private PublishTimer npcPoseTimer;
    private uint npcPoseSeq;

    void Start()
    {
        drive = GetComponent<DiffDriveController>();
        ros = ROSConnection.GetOrCreateInstance();
        try { ros.RegisterPublisher<StringMsg>(ackTopic); } catch {}
        try { ros.RegisterPublisher<PoseArrayMsg>(npcPosesTopic); } catch {}
        npcPoseTimer = new PublishTimer(npcPosesRateHz);
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

        if (seededActive)
        {
            StartDueWorkers();
            PublishNpcPoses();
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
        if (seededActive && isolateSeededScenarios)
        {
            Debug.Log("[OBSTACLE] Ignored: a seeded scenario is running and isolateSeededScenarios is on.");
            return;
        }

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
        if (msg.data != null && msg.data.IndexOf('?') >= 0)
        {
            OnSeededReset(msg.data);
            return;
        }

        // The old protocol, unchanged except that it ends seeded mode (the draggable box comes back).
        string scenarioId = msg.data.Trim().ToUpper();
        try
        {
            EndSeededMode();
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

    // ---- Seeded protocol (ADR-019) ----

    private void OnSeededReset(string text)
    {
        try
        {
            string id;
            bool hasSeed;
            int seed;
            string parseError;
            if (!ResetRequest.TryParse(text, out id, out hasSeed, out seed, out parseError))
            {
                FailSeededReset(parseError);
                return;
            }

            string path = Path.Combine(Application.streamingAssetsPath, specFileName);
            if (!File.Exists(path))
            {
                FailSeededReset("missing " + specFileName + " in StreamingAssets");
                return;
            }

            ScenarioEntry entry = ScenarioFile.FromJson(File.ReadAllText(path)).Find(id);
            if (entry == null)
            {
                FailSeededReset("unknown scenario " + id);
                return;
            }

            // Build the paths first: a bad worker spec must fail before the robot or the scene is touched.
            List<NpcPath> paths = new List<NpcPath>();
            foreach (NpcWorkerSpec w in entry.workers) paths.Add(new NpcPath(w));

            BeginSeededMode();
            ResetRobotPose();
            SetupSpecScenario(entry, paths, seed);
            SendAck("reset_scenario", true, "");
            Debug.Log($"ScenarioLoader: Reset scenario {entry.id} seed {seed} successfully ({entry.workers.Count} workers).");
        }
        catch (Exception ex)
        {
            FailSeededReset(ex.Message);
            Debug.LogError($"ScenarioLoader seeded reset error: {ex}");
        }
    }

    private void FailSeededReset(string reason)
    {
        // The ack is a hand-built JSON string, so keep quotes and line breaks out of the reason.
        string clean = (reason ?? "").Replace("\"", "'").Replace("\r", " ").Replace("\n", " ");
        SendAck("reset_scenario", false, clean);
        Debug.LogWarning("ScenarioLoader: seeded reset refused: " + clean);
    }

    private void BeginSeededMode()
    {
        seededActive = true;
        if (!isolateSeededScenarios) return;

        if (prePlacedDraggableObstacle != null && prePlacedDraggableObstacle.activeSelf)
        {
            prePlacedDraggableObstacle.SetActive(false);
            draggableHiddenBySeeded = true;
        }
        if (manualDynamicObstacle != null)
        {
            Destroy(manualDynamicObstacle);
            manualDynamicObstacle = null;
        }
    }

    private void EndSeededMode()
    {
        seededActive = false;
        specWorkers.Clear();
        if (draggableHiddenBySeeded && prePlacedDraggableObstacle != null)
        {
            prePlacedDraggableObstacle.SetActive(true);
        }
        draggableHiddenBySeeded = false;
    }

    private void SetupSpecScenario(ScenarioEntry entry, List<NpcPath> paths, int seed)
    {
        if (scenarioObstacleContainer != null)
        {
            Destroy(scenarioObstacleContainer);
        }
        scenarioObstacleContainer = new GameObject($"ScenarioObstacles_{entry.id}_seed{seed}");

        specWorkers.Clear();
        departed = false;
        resetAtS = Time.timeAsDouble;
        for (int i = 0; i < entry.workers.Count; i++)
        {
            NpcWorkerSpec spec = entry.workers[i];
            GameObject npc = GameObject.CreatePrimitive(PrimitiveType.Capsule);
            npc.name = $"{entry.id}_{spec.name}";
            SetTagSafely(npc, "NPC");
            npc.isStatic = false;
            npc.transform.SetParent(scenarioObstacleContainer.transform);
            // Same shape as the S-04 worker: 2 x radius wide in robot metres at scale 4, 2 Unity units of capsule height.
            float width = 2f * spec.radius_m * WorldScale;
            npc.transform.localScale = new Vector3(width, 2.0f, width);

            var rb = npc.AddComponent<Rigidbody>();
            rb.isKinematic = true;
            rb.useGravity = false;

            NpcMover mover = npc.AddComponent<NpcMover>();
            mover.InitFromSpec(paths[i], WorldScale, 1f);
            npc.SetActive(false); // absent until its start time

            specWorkers.Add(new SpecWorker
            {
                Spec = spec,
                Mover = mover,
                DelayS = NpcPath.StartDelay(spec, seed, i)
            });
        }
    }

    private float WorldScale { get { return transform.lossyScale.x; } }

    private void StartDueWorkers()
    {
        double now = Time.timeAsDouble;

        if (!departed && now >= resetAtS + departureArmDelayS)
        {
            Transform body = (drive != null && drive.BaseBody != null) ? drive.BaseBody : transform;
            Pose2D here = RosConversions.ToMapPose(body.position, 0f, WorldScale);
            Pose2D start = RosConversions.ToMapPose(defaultStartUnityPos, 0f, WorldScale);
            if (Math.Sqrt((here.X - start.X) * (here.X - start.X) + (here.Y - start.Y) * (here.Y - start.Y)) > departureDistanceM)
            {
                departed = true;
                foreach (SpecWorker w in specWorkers)
                {
                    if (w.Spec.start_trigger == "robot_departs") w.StartAtS = now + w.DelayS;
                }
            }
        }

        foreach (SpecWorker w in specWorkers)
        {
            if (w.Started) continue;
            if (w.Spec.start_trigger == "reset" && w.StartAtS < 0.0) w.StartAtS = resetAtS + w.DelayS;
            if (w.StartAtS >= 0.0 && now >= w.StartAtS && w.Mover != null)
            {
                w.Started = true;
                w.Mover.gameObject.SetActive(true);
            }
        }
    }

    private void PublishNpcPoses()
    {
        double now = Time.timeAsDouble;
        if (!npcPoseTimer.Tick(now)) return;

        List<PoseMsg> poses = new List<PoseMsg>();
        foreach (SpecWorker w in specWorkers)
        {
            if (!w.Started || w.Mover == null) continue;
            Vector2? p = w.Mover.RosPosition;
            if (!p.HasValue) continue;
            poses.Add(new PoseMsg(new PointMsg(p.Value.x, p.Value.y, 0.0), RosConversions.YawToQuaternion(w.Mover.RosYaw)));
        }

        HeaderMsg header = new HeaderMsg(npcPoseSeq++, RosConversions.ToRosTime(now), "map");
        try { ros.Publish(npcPosesTopic, new PoseArrayMsg(header, poses.ToArray())); }
        catch (Exception ex) { Debug.LogWarning($"npc_poses publish error: {ex.Message}"); }
    }

    private void ResetRobotPose()
    {
        if (drive == null)
        {
            drive = GetComponent<DiffDriveController>();
        }

        if (drive != null)
        {
            drive.Stop();
        }

        transform.position = defaultStartUnityPos;
        transform.rotation = Quaternion.Euler(defaultStartUnityEuler);

        ArticulationBody rootAb = null;
        if (drive != null && drive.BaseBody != null)
        {
            rootAb = drive.BaseBody.GetComponent<ArticulationBody>();
        }

        if (rootAb == null)
        {
            Transform link = transform.Find("base_footprint/base_link") ?? transform.Find("base_link");
            if (link != null)
            {
                rootAb = link.GetComponent<ArticulationBody>();
            }
        }

        if (rootAb == null)
        {
            foreach (ArticulationBody body in GetComponentsInChildren<ArticulationBody>())
            {
                Transform parent = body.transform.parent;
                if (parent == null || parent.GetComponentInParent<ArticulationBody>() == null)
                {
                    rootAb = body;
                    break;
                }
            }
        }

        if (rootAb != null)
        {
            rootAb.velocity = Vector3.zero;
            rootAb.angularVelocity = Vector3.zero;
            rootAb.TeleportRoot(defaultStartUnityPos, Quaternion.Euler(defaultStartUnityEuler));

            foreach (ArticulationBody body in GetComponentsInChildren<ArticulationBody>())
            {
                body.velocity = Vector3.zero;
                body.angularVelocity = Vector3.zero;
            }
        }
        else
        {
            var rb = GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.velocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            }
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
