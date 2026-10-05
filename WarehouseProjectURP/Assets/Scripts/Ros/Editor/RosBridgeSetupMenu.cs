using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

/// <summary>
/// Menu "Robotics > Warehouse > Add ROS Bridge": adds the ROS publishers/subscriber to the robot (the object with
/// DiffDriveController) and one ClockPublisher to the scene. Safe to run twice; it only adds what is missing.
///
/// It also puts the scene in ROS mode: the robot's TF is published once (RobotPosePublisher owns map -> base_footprint,
/// so OdometryPublisher's TF is switched off) and the Unity-only TurtleBotNavigator no longer starts a mission
/// by itself (ROS drives the robot).
///
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
        added += AddIfMissing<RobotPosePublisher>(robot);
        added += AddIfMissing<WarehouseMapPublisher>(robot);
        added += AddIfMissing<CollisionReporter>(robot);

        if (Object.FindObjectOfType<ClockPublisher>() == null)
        {
            GameObject clock = new GameObject(ClockObjectName);
            Undo.RegisterCreatedObjectUndo(clock, "Add RosClock");
            Undo.AddComponent<ClockPublisher>(clock);
            added++;
        }

        string changed = SetRosMode(robot);

        EditorSceneManager.MarkSceneDirty(robot.scene);
        Selection.activeGameObject = robot;
        Debug.Log("Add ROS Bridge: " + added + " component(s) added to '" + robot.name + "' / '" + ClockObjectName + "'." +
                  (changed.Length > 0 ? "\n" + changed : "") + "\nSave the scene (Ctrl+S).", robot);
    }

    /// <summary>One TF parent for base_footprint, and no mission that starts by itself. Returns what it changed.</summary>
    private static string SetRosMode(GameObject robot)
    {
        string changed = "";

        OdometryPublisher odometry = robot.GetComponent<OdometryPublisher>();
        RobotPosePublisher pose = robot.GetComponent<RobotPosePublisher>();
        if (odometry != null && odometry.publishTf && pose != null && pose.publishTf)
        {
            Undo.RecordObject(odometry, "Switch off odometry TF");
            odometry.publishTf = false;
            changed += "  OdometryPublisher.publishTf switched off (RobotPosePublisher publishes map -> base_footprint).\n";
        }

        TurtleBotNavigator navigator = robot.GetComponent<TurtleBotNavigator>();
        if (navigator != null && navigator.autoStart)
        {
            Undo.RecordObject(navigator, "Switch off Auto Start");
            navigator.autoStart = false;
            changed += "  TurtleBotNavigator.autoStart switched off (ROS drives the robot).\n";
        }
        return changed;
    }

    private static int AddIfMissing<T>(GameObject go) where T : Component
    {
        if (go.GetComponent<T>() != null) return 0;
        Undo.AddComponent<T>(go);
        return 1;
    }
}
