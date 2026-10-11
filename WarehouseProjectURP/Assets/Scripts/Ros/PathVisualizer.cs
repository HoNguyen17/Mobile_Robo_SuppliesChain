using System;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Visually renders the ROS /planned_path directly on the warehouse floor in Unity using a glowing LineRenderer,
/// and draws a visual Goal Ring at the destination.
/// 
/// Updates in real-time when A* replans to avoid dynamic obstacles!
/// </summary>
[DisallowMultipleComponent]
public class PathVisualizer : MonoBehaviour
{
    public string pathTopic = "/planned_path";
    public string goalTopic = "/move_base_simple/goal";

    [Tooltip("World scale factor (robot meters to Unity units).")]
    public float worldScale = 4.0f;

    [Tooltip("Height above floor to render path to avoid z-fighting.")]
    public float floorYOffset = 0.08f;

    [Tooltip("Line width of the path.")]
    public float lineWidth = 0.35f;

    private ROSConnection ros;
    private LineRenderer lineRenderer;
    private GameObject goalMarker;

    void Awake()
    {
        SetupLineRenderer();
        SetupGoalMarker();
    }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<PathMsg>(pathTopic, OnPathReceived);
        ros.Subscribe<PoseStampedMsg>(goalTopic, OnGoalReceived);
    }

    void OnDestroy()
    {
        if (ros != null)
        {
            ros.Unsubscribe(pathTopic);
            ros.Unsubscribe(goalTopic);
        }
    }

    private void SetupLineRenderer()
    {
        lineRenderer = GetComponent<LineRenderer>();
        if (lineRenderer == null)
        {
            lineRenderer = gameObject.AddComponent<LineRenderer>();
        }

        lineRenderer.useWorldSpace = true;
        lineRenderer.startWidth = lineWidth;
        lineRenderer.endWidth = lineWidth;
        lineRenderer.positionCount = 0;
        lineRenderer.numCapVertices = 4;
        lineRenderer.numCornerVertices = 4;

        Shader shader = Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Sprites/Default");
        if (shader != null)
        {
            Material mat = new Material(shader);
            mat.color = new Color(0f, 0.9f, 0.5f, 0.85f); // Bright Neon Cyan/Green
            lineRenderer.material = mat;
        }

        Gradient gradient = new Gradient();
        gradient.SetKeys(
            new GradientColorKey[] {
                new GradientColorKey(new Color(0f, 0.85f, 1f), 0.0f), // Cyan start
                new GradientColorKey(new Color(0.2f, 1f, 0.4f), 1.0f) // Neon Green end
            },
            new GradientAlphaKey[] {
                new GradientAlphaKey(0.85f, 0.0f),
                new GradientAlphaKey(0.85f, 1.0f)
            }
        );
        lineRenderer.colorGradient = gradient;
    }

    private void SetupGoalMarker()
    {
        goalMarker = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        goalMarker.name = "GoalMarker_Visual";
        goalMarker.transform.SetParent(transform);
        goalMarker.transform.localScale = new Vector3(1.2f, 0.04f, 1.2f); // Flat disc

        // Remove collider so it doesn't trigger collisions
        Collider col = goalMarker.GetComponent<Collider>();
        if (col != null) Destroy(col);

        Shader shader = Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Sprites/Default");
        if (shader != null)
        {
            Material mat = new Material(shader);
            mat.color = new Color(1f, 0.8f, 0.0f, 0.75f); // Glowing Yellow / Amber
            goalMarker.GetComponent<Renderer>().material = mat;
        }

        goalMarker.SetActive(false);
    }

    private void OnPathReceived(PathMsg msg)
    {
        if (msg == null || msg.poses == null || msg.poses.Length == 0)
        {
            lineRenderer.positionCount = 0;
            return;
        }

        int count = msg.poses.Length;
        Vector3[] positions = new Vector3[count];

        for (int i = 0; i < count; i++)
        {
            double rx = msg.poses[i].pose.position.x;
            double ry = msg.poses[i].pose.position.y;

            // Convert ROS (map frame) to Unity:
            // ROS X forward -> Unity +Z
            // ROS Y left    -> Unity -X
            float ux = -(float)ry * worldScale;
            float uz = (float)rx * worldScale;
            float uy = floorYOffset;

            positions[i] = new Vector3(ux, uy, uz);
        }

        lineRenderer.positionCount = count;
        lineRenderer.SetPositions(positions);

        if (goalMarker != null && count > 0)
        {
            Vector3 lastPos = positions[count - 1];
            goalMarker.transform.position = new Vector3(lastPos.x, floorYOffset + 0.02f, lastPos.z);
            goalMarker.SetActive(true);
        }

        Debug.Log($"PathVisualizer: Rendered path with {count} waypoints on warehouse floor.");
    }

    private void OnGoalReceived(PoseStampedMsg msg)
    {
        if (msg == null) return;

        double rx = msg.pose.position.x;
        double ry = msg.pose.position.y;

        float ux = -(float)ry * worldScale;
        float uz = (float)rx * worldScale;
        float uy = floorYOffset + 0.02f;

        if (goalMarker != null)
        {
            goalMarker.transform.position = new Vector3(ux, uy, uz);
            goalMarker.SetActive(true);
        }
    }

    public void ClearVisuals()
    {
        if (lineRenderer != null) lineRenderer.positionCount = 0;
        if (goalMarker != null) goalMarker.SetActive(false);
    }
}
