using System.Collections;
using NUnit.Framework;
#if UNITY_EDITOR
using UnityEditor.SceneManagement;
#endif
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

/// <summary>
/// Holding brake on the real Warehouse scene (ADR-011 gravity): with no command the robot must not creep,
/// and a new command must still drive it.
/// </summary>
public class RobotHoldTests
{
    private const float SettleSeconds = 3f;
    private const float IdleSeconds = 30f;
    private const float MaxIdleDriftRealM = 0.005f; // 5 mm in 30 s (before the brake: ~50 mm)
    private const float DriveSeconds = 2f;

    [UnityTest]
    public IEnumerator Robot_holds_still_then_drives_again()
    {
#if UNITY_EDITOR
        yield return EditorSceneManager.LoadSceneAsyncInPlayMode("Assets/Scenes/Warehouse.unity", new LoadSceneParameters(LoadSceneMode.Single));
#else
        yield break;
#endif
        yield return null;

        DiffDriveController drive = Object.FindObjectOfType<DiffDriveController>();
        Assert.IsNotNull(drive, "no DiffDriveController in the scene");
        TurtleBotNavigator nav = drive.GetComponent<TurtleBotNavigator>();
        if (nav != null) nav.enabled = false; // nothing may send commands during the idle phase
        float scale = drive.transform.lossyScale.x;
        Transform body = drive.BaseBody;

        yield return new WaitForSeconds(SettleSeconds);
        Vector3 start = body.position;
        yield return new WaitForSeconds(IdleSeconds);
        float driftRealM = Horizontal(body.position - start) / scale;
        Debug.Log("[HoldTest] idle drift over " + IdleSeconds + " s: " + (driftRealM * 1000f).ToString("F2") + " mm (real)");
        Assert.Less(driftRealM, MaxIdleDriftRealM, "robot creeps while stopped");

        Vector3 beforeDrive = body.position;
        float end = Time.time + DriveSeconds;
        while (Time.time < end)
        {
            drive.SetCommand(0.2f, 0f);
            yield return new WaitForFixedUpdate();
        }
        drive.Stop();
        float drivenRealM = Horizontal(body.position - beforeDrive) / scale;
        Debug.Log("[HoldTest] drove " + drivenRealM.ToString("F3") + " m (real) in " + DriveSeconds + " s at 0.2 m/s");
        Assert.Greater(drivenRealM, 0.25f, "the brake did not release for a new command");
    }

    private static float Horizontal(Vector3 v)
    {
        v.y = 0f;
        return v.magnitude;
    }
}
