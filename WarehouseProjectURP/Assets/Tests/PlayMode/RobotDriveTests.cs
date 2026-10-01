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
/// Loads the real Warehouse scene in Play mode, lets the TurtleBot3 settle under gravity, then
/// drives it through DiffDriveController and measures the motion. Results go to
/// test_results.txt two folders above Assets.
/// </summary>
public class RobotDriveTests
{
    private const string ScenePath = "Assets/Scenes/Warehouse.unity";
    private const float SettleSeconds = 3f;
    private const float DriveSeconds = 5f;
    private const float TurnSeconds = 3f;

    private readonly StringBuilder report = new StringBuilder();
    private readonly System.Collections.Generic.List<string> failures = new System.Collections.Generic.List<string>();

    private void Log(string s)
    {
        report.AppendLine(s);
        Debug.Log("[RobotTest] " + s);
    }

    private static Transform FindDeep(Transform root, string name)
    {
        foreach (Transform t in root.GetComponentsInChildren<Transform>(true))
            if (t.name == name) return t;
        return null;
    }

    private static Component Get(GameObject go, string typeName)
    {
        foreach (Component c in go.GetComponents<Component>())
            if (c != null && c.GetType().Name == typeName) return c;
        return null;
    }

    private static void Call(Component c, string method, params object[] args)
    {
        MethodInfo m = c.GetType().GetMethod(method, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
        if (m == null) throw new MissingMethodException(c.GetType().Name, method);
        m.Invoke(c, args);
    }

    private static float Yaw(Transform t)
    {
        Vector3 f = t.forward; f.y = 0f;
        return Mathf.Atan2(f.x, f.z) * Mathf.Rad2Deg;
    }

    private void Dump(GameObject root, Transform baseFootprint)
    {
        Log("scene=" + SceneManager.GetActiveScene().name + " gravity=" + Physics.gravity +
            " fixedDt=" + Time.fixedDeltaTime + " solverIters=" + Physics.defaultSolverIterations);
        Log("root scale=" + root.transform.lossyScale + " pos=" + root.transform.position);
        Log("base_footprint pos=" + baseFootprint.position + " up.y=" + baseFootprint.up.y);
        foreach (ArticulationBody ab in root.GetComponentsInChildren<ArticulationBody>())
        {
            Log("AB " + ab.name + " root=" + ab.isRoot + " immovable=" + ab.immovable + " gravity=" + ab.useGravity +
                " mass=" + ab.mass + " joint=" + ab.jointType + " twistLock=" + ab.twistLock +
                " drive(k=" + ab.xDrive.stiffness + ",d=" + ab.xDrive.damping + ",F=" + ab.xDrive.forceLimit + ")" +
                " lin/angDamp=" + ab.linearDamping + "/" + ab.angularDamping + " friction=" + ab.jointFriction);
        }
        // Ground check: nearest collider below the robot that is NOT part of the robot itself.
        RaycastHit[] hits = Physics.RaycastAll(baseFootprint.position + Vector3.up * 5f, Vector3.down, 60f);
        Array.Sort(hits, (a, b) => a.distance.CompareTo(b.distance));
        bool found = false;
        foreach (RaycastHit h in hits)
        {
            if (h.collider.transform.IsChildOf(root.transform)) continue;
            Log("ground below: " + h.collider.name + " (parent " + (h.collider.transform.parent != null ? h.collider.transform.parent.name : "-") + ") at y=" + h.point.y);
            found = true;
            break;
        }
        if (!found) Log("ground below: NOTHING (no collider under the robot)");
        foreach (Collider c in root.GetComponentsInChildren<Collider>())
        {
            Log("robot collider " + c.name + " on " + c.transform.parent.name + " type=" + c.GetType().Name + " trigger=" + c.isTrigger + " bounds.min.y=" + c.bounds.min.y.ToString("F3") + " size=" + c.bounds.size);
        }
    }

    private IEnumerator Drive(Component drive, float linear, float angular, float seconds)
    {
        float end = Time.time + seconds;
        while (Time.time < end)
        {
            Call(drive, "SetCommand", linear, angular);
            yield return new WaitForFixedUpdate();
        }
        Call(drive, "SetCommand", 0f, 0f);
    }

    [UnityTest]
    public IEnumerator Drives_on_warehouse_scene() { yield return Run("Assets/Scenes/Warehouse.unity"); }

    private IEnumerator Run(string scenePath)
    {
        report.Clear();
        failures.Clear();
        Log("===== " + scenePath + " =====");
#if UNITY_EDITOR
        yield return EditorSceneManager.LoadSceneAsyncInPlayMode(scenePath, new LoadSceneParameters(LoadSceneMode.Single));
#else
        yield break;
#endif
        yield return null;
        yield return null;

        GameObject root = GameObject.Find("turtlebot3_waffle_pi");
        Assert.IsNotNull(root, "robot 'turtlebot3_waffle_pi' not found in the scene");
        Transform baseFootprint = null;
        foreach (ArticulationBody ab in root.GetComponentsInChildren<ArticulationBody>())
            if (ab.transform.parent == null || ab.transform.parent.GetComponentInParent<ArticulationBody>() == null) { baseFootprint = ab.transform; break; }
        Assert.IsNotNull(baseFootprint, "articulation root not found");

        Component drive = Get(root, "DiffDriveController");
        Component nav = Get(root, "TurtleBotNavigator");
        Assert.IsNotNull(drive, "DiffDriveController is not on the robot root");
        if (nav != null) ((Behaviour)nav).enabled = false;

        float scale = root.transform.lossyScale.x;
        Dump(root, baseFootprint);

        // ---- 1. gravity settle ----
        Vector3 p0 = baseFootprint.position;
        float minUp = 1f;
        float end = Time.time + SettleSeconds;
        while (Time.time < end)
        {
            minUp = Mathf.Min(minUp, baseFootprint.up.y);
            yield return new WaitForSeconds(0.5f);
            Log("settle t=" + (SettleSeconds - (end - Time.time)).ToString("F1") + " pos=" + baseFootprint.position + " up.y=" + baseFootprint.up.y);
        }
        Vector3 pSettled = baseFootprint.position;
        foreach (Collider c in root.GetComponentsInChildren<Collider>())
            if (c.enabled) Log("  after-settle collider " + c.GetType().Name + " " + c.name + " min.y=" + c.bounds.min.y.ToString("F4") + " centre.y=" + c.bounds.center.y.ToString("F4") + " mat=" + (c.sharedMaterial != null ? c.sharedMaterial.name + "(" + c.sharedMaterial.dynamicFriction + ")" : "none"));
        foreach (Collider c in root.GetComponentsInChildren<Collider>())
        {
            if (!c.enabled) continue;
            foreach (Collider h in Physics.OverlapBox(c.bounds.center, c.bounds.extents * 1.02f))
            {
                if (h.transform.IsChildOf(root.transform)) continue;
                Log("  OVERLAPS " + c.name + " (layer " + c.gameObject.layer + ") with " + h.GetType().Name + " " + h.name + " parent=" + (h.transform.parent != null ? h.transform.parent.name : "-") + " layer=" + h.gameObject.layer + " bounds.y=[" + h.bounds.min.y.ToString("F3") + "," + h.bounds.max.y.ToString("F3") + "] ignoreColl=" + Physics.GetIgnoreCollision(c, h) + " ignoreLayer=" + Physics.GetIgnoreLayerCollision(c.gameObject.layer, h.gameObject.layer) + " trigger=" + h.isTrigger);
            }
        }
        Log("  base pos=" + baseFootprint.position.ToString("F4") + " euler=" + baseFootprint.eulerAngles.ToString("F1"));
        Log("SETTLE dy=" + (pSettled.y - p0.y).ToString("F3") + " minUp.y=" + minUp.ToString("F3") + " NaN=" + float.IsNaN(pSettled.x));
        if (float.IsNaN(pSettled.x)) failures.Add("robot position is NaN after settling");
        if (pSettled.y < p0.y - 1f) failures.Add("robot fell through the floor (dy=" + (pSettled.y - p0.y) + ")");
        if (baseFootprint.up.y < 0.9f) failures.Add("robot tipped over (up.y=" + baseFootprint.up.y + ")");

        // ---- 2. forward ----
        Vector3 f0 = baseFootprint.position;
        Vector3 fwdDir = baseFootprint.forward; fwdDir.y = 0f; fwdDir.Normalize();
        ArticulationBody fl = null, fr = null;
        foreach (ArticulationBody ab in root.GetComponentsInChildren<ArticulationBody>()) { if (ab.name == "wheel_left_link") fl = ab; if (ab.name == "wheel_right_link") fr = ab; }
        float fEnd = Time.time + DriveSeconds; bool fSampled = false;
        while (Time.time < fEnd)
        {
            Call(drive, "SetCommand", 0.2f, 0f);
            yield return new WaitForFixedUpdate();
            if (!fSampled && Time.time > fEnd - 2.5f) { fSampled = true; Log("FORWARD mid: wheel jointVelocity L=" + fl.jointVelocity[0].ToString("F2") + " R=" + fr.jointVelocity[0].ToString("F2") + " rad/s (wanted 6.06), base euler=" + baseFootprint.eulerAngles.ToString("F1")); }
        }
        Call(drive, "SetCommand", 0f, 0f);
        Vector3 f1 = baseFootprint.position;
        Vector3 delta = f1 - f0; delta.y = 0f;
        float alongMetres = Vector3.Dot(delta, fwdDir) / scale;
        float totalMetres = delta.magnitude / scale;
        Log("FORWARD cmd=0.2 m/s for " + DriveSeconds + "s: expected ~" + (0.2f * DriveSeconds) + " m, along=" + alongMetres.ToString("F3") +
            " m, total=" + totalMetres.ToString("F3") + " m, up.y=" + baseFootprint.up.y.ToString("F3"));
        if (totalMetres < 0.4f * 0.2f * DriveSeconds) failures.Add("robot barely moved: " + totalMetres + " m (expected ~" + 0.2f * DriveSeconds + ")");
        else if (alongMetres < 0f) failures.Add("robot drove BACKWARD (along=" + alongMetres + ") -> invert both wheels");
        else if (alongMetres < 0.7f * 0.2f * DriveSeconds) failures.Add("forward distance too short: " + alongMetres);
        if (baseFootprint.up.y < 0.9f) failures.Add("robot tipped over while driving");

        yield return new WaitForSeconds(1f);

        // ---- 3. turn left (positive angular = counter-clockwise) ----
        float y0 = Yaw(baseFootprint);
        Vector3 t0 = baseFootprint.position;
        ArticulationBody wl = null, wr = null;
        foreach (ArticulationBody ab in root.GetComponentsInChildren<ArticulationBody>()) { if (ab.name == "wheel_left_link") wl = ab; if (ab.name == "wheel_right_link") wr = ab; }
        float turnEnd = Time.time + TurnSeconds;
        bool turnSampled = false;
        while (Time.time < turnEnd)
        {
            Call(drive, "SetCommand", 0f, 1.0f);
            yield return new WaitForFixedUpdate();
            if (!turnSampled && Time.time > turnEnd - 1.5f)
            {
                turnSampled = true;
                Log("TURN mid: wheel jointVelocity L=" + wl.jointVelocity[0].ToString("F2") + " R=" + wr.jointVelocity[0].ToString("F2") + " rad/s (wanted -/+4.36), yaw so far=" + Mathf.DeltaAngle(y0, Yaw(baseFootprint)).ToString("F1") + " deg, up.y=" + baseFootprint.up.y.ToString("F3"));
            }
        }
        Call(drive, "SetCommand", 0f, 0f);
        float y1 = Yaw(baseFootprint);
        float dYaw = Mathf.DeltaAngle(y0, y1); // Unity yaw grows clockwise from above
        float drift = (baseFootprint.position - t0).magnitude / scale;
        Log("TURN cmd=+1.0 rad/s for " + TurnSeconds + "s: expected ~" + (-1.0f * TurnSeconds * Mathf.Rad2Deg).ToString("F0") +
            " deg (left = negative Unity yaw), got " + dYaw.ToString("F1") + " deg, drift=" + drift.ToString("F3") + " m");
        if (Mathf.Abs(dYaw) < 20f) failures.Add("robot barely turned: " + dYaw + " deg");
        else if (dYaw > 0f) failures.Add("robot turned RIGHT for a positive (left) command -> swap/invert a wheel");

        Log("RESULT " + (failures.Count == 0 ? "PASS" : "FAIL: " + string.Join(" | ", failures)));
        File.AppendAllText(Path.Combine(Application.dataPath, "../test_results.txt"), report.ToString());
        Assert.IsEmpty(failures, string.Join("\n", failures));
    }
}
