using System.Collections.Generic;
using System.Text;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// Menu "Robotics > Warehouse > Apply Physics Standard" (ADR-011):
///   1. sets the project gravity to PhysicsStandard.UnityGravity,
///   2. declares a PhysicalBody on the warehouse shell (static concrete), stations (static plastic),
///      every rack (static steel) and every box (dynamic cardboard, Standard mass),
///   3. writes the resulting masses and flags into the scene so the Inspector shows them,
///   4. lists any physics object it could not classify.
/// Safe to run again. Save the scene afterwards; then run the EditMode tests (WarehousePhysicsSceneTests).
/// </summary>
public static class PhysicsStandardMenu
{
    private const string ShellPath = "GeneratedWarehouse/Warehouse";
    private const string StationsPath = "GeneratedWarehouse/Stations";
    private const string RackName = "ShelvingRackRandom(Clone)";
    private const string BoxNamePrefix = "box";

    [MenuItem("Robotics/Warehouse/Apply Physics Standard")]
    public static void ApplyPhysicsStandard()
    {
        SetProjectGravity();

        Scene scene = SceneManager.GetActiveScene();
        DiffDriveController drive = Object.FindObjectOfType<DiffDriveController>();
        Transform robot = drive != null ? drive.transform : null;

        int declared = 0;
        declared += Declare(GameObject.Find(ShellPath), true, SurfaceMaterial.Concrete, 0f);
        declared += Declare(GameObject.Find(StationsPath), true, SurfaceMaterial.Plastic, 0f);

        List<Transform> all = AllTransforms(scene);
        foreach (Transform t in all)
        {
            if (t.name == RackName) declared += Declare(t.gameObject, true, SurfaceMaterial.Steel, 0f);
        }
        float standardBox = PhysicsStandard.BoxRealMassKg(BoxCategory.Standard);
        foreach (Transform t in all)
        {
            if (t.GetComponent<Rigidbody>() == null || IsUnder(t, robot)) continue;
            if (t.name.ToLowerInvariant().StartsWith(BoxNamePrefix))
            {
                declared += Declare(t.gameObject, false, SurfaceMaterial.Cardboard, standardBox);
            }
        }

        foreach (PhysicalBody body in Object.FindObjectsOfType<PhysicalBody>())
        {
            Undo.RecordObjects(body.GetComponentsInChildren<Rigidbody>(true), "Apply Physics Standard");
            body.ApplyBody();
        }

        EditorSceneManager.MarkSceneDirty(scene);
        string leftovers = Unclassified(all, robot);
        Debug.Log("Apply Physics Standard: gravity " + (-PhysicsStandard.UnityGravity) + " m/s^2, " + declared +
                  " PhysicalBody added. Save the scene (Ctrl+S)." +
                  (leftovers.Length > 0 ? "\nNot classified (add a PhysicalBody by hand):\n" + leftovers : ""));
    }

    private static void SetProjectGravity()
    {
        Object[] assets = AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/DynamicsManager.asset");
        if (assets.Length == 0)
        {
            Debug.LogError("Apply Physics Standard: ProjectSettings/DynamicsManager.asset not found.");
            return;
        }
        SerializedObject settings = new SerializedObject(assets[0]);
        settings.FindProperty("m_Gravity").vector3Value = new Vector3(0f, -PhysicsStandard.UnityGravity, 0f);
        settings.ApplyModifiedProperties();
        AssetDatabase.SaveAssets();
    }

    /// <summary>Adds or updates a PhysicalBody. Returns 1 if a new one was added.</summary>
    private static int Declare(GameObject go, bool isStatic, SurfaceMaterial material, float realMassKg)
    {
        if (go == null) return 0;
        PhysicalBody body = go.GetComponent<PhysicalBody>();
        bool added = body == null;
        if (added) body = Undo.AddComponent<PhysicalBody>(go);
        else Undo.RecordObject(body, "Apply Physics Standard");

        body.isStatic = isStatic;
        body.material = material;
        if (!isStatic && (added || body.realMassKg <= 0f)) body.realMassKg = realMassKg; // keep a mass set by hand
        EditorUtility.SetDirty(body);
        return added ? 1 : 0;
    }

    private static string Unclassified(List<Transform> all, Transform robot)
    {
        StringBuilder sb = new StringBuilder();
        int count = 0;
        foreach (Transform t in all)
        {
            if (IsUnder(t, robot) || PhysicalBody.OwnerOf(t) != null) continue;
            Collider c = t.GetComponent<Collider>();
            bool physical = t.GetComponent<Rigidbody>() != null || (c != null && !c.isTrigger);
            if (physical && count++ < 20) sb.AppendLine("  " + t.name);
        }
        return sb.ToString();
    }

    private static List<Transform> AllTransforms(Scene scene)
    {
        List<Transform> list = new List<Transform>();
        foreach (GameObject root in scene.GetRootGameObjects())
        {
            list.AddRange(root.GetComponentsInChildren<Transform>(true));
        }
        return list;
    }

    private static bool IsUnder(Transform t, Transform root)
    {
        return root != null && t.IsChildOf(root);
    }
}
