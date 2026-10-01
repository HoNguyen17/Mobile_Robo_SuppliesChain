using System;
using System.Collections;
using System.IO;
using System.Reflection;
using System.Text;
using NUnit.Framework;
#if UNITY_EDITOR
using UnityEditor.SceneManagement;
#endif
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

/// <summary>
/// End to end: the saved Warehouse scene (navigator with Auto Start ticked) must drive the robot to the
/// target shelf, dwell, drive home and stop, without tipping over.
/// </summary>
public class RobotNavigatorTests
{
    private readonly StringBuilder report = new StringBuilder();
    private void Log(string s) { report.AppendLine(s); Debug.Log("[NavTest] " + s); }

    private static Component Get(GameObject go, string typeName)
    {
        foreach (Component c in go.GetComponents<Component>())
            if (c != null && c.GetType().Name == typeName) return c;
        return null;
    }

    private static object Prop(Component c, string name) { return c.GetType().GetProperty(name).GetValue(c); }
    private static object Field(Component c, string name) { return c.GetType().GetField(name).GetValue(c); }

    [UnityTest]
    [Timeout(900000)]
    public IEnumerator Navigator_goes_to_shelf_and_home()
    {
        report.Clear();
#if UNITY_EDITOR
        yield return EditorSceneManager.LoadSceneAsyncInPlayMode("Assets/Scenes/Warehouse.unity", new LoadSceneParameters(LoadSceneMode.Single));
#else
        yield break;
#endif
        yield return null;
        yield return null;

        GameObject root = GameObject.Find("turtlebot3_waffle_pi");
        Assert.IsNotNull(root);
        Component nav = Get(root, "TurtleBotNavigator");
        Component drive = Get(root, "DiffDriveController");
        Assert.IsNotNull(nav, "TurtleBotNavigator missing on the robot");
        Transform body = (Transform)Prop(drive, "BaseBody");
        Transform target = (Transform)Field(nav, "target");
        Transform home = (Transform)Field(nav, "homePoint");
        Assert.IsNotNull(target, "navigator target is not assigned in the scene");
        Log("autoStart=" + Field(nav, "autoStart") + " target=" + target.name + " at " + target.position + " home=" + (home != null ? home.position.ToString() : "spawn") + " start=" + body.position);

        bool sawTarget = false, sawHome = false, tipped = false;
        string last = "";
        float minDistTarget = float.MaxValue, minUp = 1f;
        float t0 = Time.time;
        float nextLog = 0f;
        Vector3 spawn = body.position;
        while (Time.time - t0 < 300f)
        {
            yield return new WaitForFixedUpdate();
            string state = Prop(nav, "State").ToString();
            minUp = Mathf.Min(minUp, body.up.y);
            Vector3 d = target.position - body.position; d.y = 0f;
            minDistTarget = Mathf.Min(minDistTarget, d.magnitude);
            if (state == "WaitingAtTarget") sawTarget = true;
            if (state == "DrivingHome") sawHome = true;
            if (state != last) { Log("t=" + (Time.time - t0).ToString("F1") + " state -> " + state + " pos=" + body.position.ToString("F2")); last = state; }
            if (Time.time - t0 >= nextLog) { nextLog += 10f; Log("  t=" + (Time.time - t0).ToString("F0") + " state=" + state + " pos=" + body.position.ToString("F2") + " up.y=" + body.up.y.ToString("F2") + " distToTarget=" + d.magnitude.ToString("F2")); }
            if (sawHome && state == "Idle") break;
        }

        Vector3 homePos = home != null ? home.position : spawn;
        Vector3 dh = homePos - body.position; dh.y = 0f;
        Log("END state=" + last + " sawTarget=" + sawTarget + " sawHome=" + sawHome + " minDistToTarget=" + minDistTarget.ToString("F2") +
            " finalDistToHome=" + dh.magnitude.ToString("F2") + " minUp.y=" + minUp.ToString("F2"));

        File.AppendAllText(Path.Combine(Application.dataPath, "../nav_results.txt"), report.ToString());
        Assert.IsTrue(sawTarget, "robot never reached the target shelf");
        Assert.IsTrue(sawHome && last == "Idle", "robot did not finish driving home (last state " + last + ")");
        Assert.Greater(minUp, 0.8f, "robot tipped over during the mission");
        Assert.Less(dh.magnitude, 3f, "robot did not stop near home");
    }
}
