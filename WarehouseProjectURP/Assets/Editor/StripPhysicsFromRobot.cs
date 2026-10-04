using UnityEditor;
using UnityEngine;

// Editor tool: turns an imported URDF robot into a visual-only model
// (keeps meshes, removes ArticulationBody, colliders and URDF helper components).
public static class StripPhysicsFromRobot
{
    [MenuItem("Tools/Robot/Strip Physics (Visual Only)")]
    public static void Strip()
    {
        GameObject root = Selection.activeGameObject;
        if (root == null)
        {
            Debug.LogError("StripPhysics: select the imported robot in the Hierarchy first.");
            return;
        }

        int urdfRemoved = 0, bodiesRemoved = 0, collidersRemoved = 0;

        // Children first, so parents are never removed while children still depend on them.
        Transform[] all = root.GetComponentsInChildren<Transform>(true);
        System.Array.Reverse(all);

        // Pass 1: URDF helper components (they may require ArticulationBody/colliders).
        // Several rounds, because helpers can depend on each other.
        for (int round = 0; round < 5; round++)
        {
            int removedThisRound = 0;
            foreach (Transform t in all)
            {
                foreach (MonoBehaviour mb in t.GetComponents<MonoBehaviour>())
                {
                    if (mb == null) continue;
                    string ns = mb.GetType().Namespace;
                    if (ns != null && ns.StartsWith("Unity.Robotics.UrdfImporter"))
                    {
                        Undo.DestroyObjectImmediate(mb);
                        urdfRemoved++;
                        removedThisRound++;
                    }
                }
            }
            if (removedThisRound == 0) break;
        }

        // Pass 2: ArticulationBody
        foreach (Transform t in all)
        {
            foreach (ArticulationBody ab in t.GetComponents<ArticulationBody>())
            {
                Undo.DestroyObjectImmediate(ab);
                bodiesRemoved++;
            }
        }

        // Pass 3: colliders
        foreach (Transform t in all)
        {
            foreach (Collider c in t.GetComponents<Collider>())
            {
                Undo.DestroyObjectImmediate(c);
                collidersRemoved++;
            }
        }

        Debug.Log("StripPhysics done on '" + root.name + "': " + urdfRemoved +
                  " URDF components, " + bodiesRemoved + " ArticulationBodies, " +
                  collidersRemoved + " colliders removed.");
    }
}
