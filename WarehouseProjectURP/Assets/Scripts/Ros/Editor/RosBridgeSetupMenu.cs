using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

/// <summary>
/// Menu "Robotics > Warehouse > Add ROS Bridge": adds the ROS publishers/subscriber to the robot (the object with
/// DiffDriveController) and one ClockPublisher to the scene. Safe to run twice; it only adds what is missing.
/// Save the scene afterwards.
/// </summary>
public static class RosBridgeSetupMenu
{
    private const string ClockObjectName = "RosClock";

    [MenuItem("Robotics/Warehouse/Add ROS Bridge")]
    public static void AddRosBridge()
    {
        DiffDriveController drive = Object.FindObjectOfType<DiffDriveController>();
        if (drive == null)
        {
            EditorUtility.DisplayDialog("Add ROS Bridge", "No DiffDriveController found in the open scene.", "OK");
            return;
        }

        GameObject robot = drive.gameObject;
        int added = 0;
        added += AddIfMissing<OdometryPublisher>(robot);
        added += AddIfMissing<LaserScanPublisher>(robot);
        added += AddIfMissing<CmdVelSubscriber>(robot);

        if (Object.FindObjectOfType<ClockPublisher>() == null)
        {
            GameObject clock = new GameObject(ClockObjectName);
            Undo.RegisterCreatedObjectUndo(clock, "Add RosClock");
            Undo.AddComponent<ClockPublisher>(clock);
            added++;
        }

        EditorSceneManager.MarkSceneDirty(robot.scene);
        Selection.activeGameObject = robot;
        Debug.Log("Add ROS Bridge: " + added + " component(s) added to '" + robot.name + "' / '" + ClockObjectName +
                  "'. Save the scene (Ctrl+S).", robot);
    }

    private static int AddIfMissing<T>(GameObject go) where T : Component
    {
        if (go.GetComponent<T>() != null) return 0;
        Undo.AddComponent<T>(go);
        return 1;
    }
}
