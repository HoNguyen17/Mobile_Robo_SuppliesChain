using System.Collections;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;

/// <summary>
/// Publishes the occupancy grid built by CubeCarNavigator as a ROS nav_msgs/OccupancyGrid on /map.
///
/// Attach to the Cube (same GameObject as CubeCarNavigator). 'Cube Car Navigator' may stay
/// unticked: this script only asks it to build its grid, it never asks it to drive.
///
/// Frame conversion (same mapping as CubePosePublisher):
///   ROS x = Unity Z        ROS y = -Unity X
/// so the Unity grid is transposed and flipped before it is sent (see BuildMessage).
/// The ROS frame origin is the Unity world origin, so no offset is needed.
/// </summary>
public class OccupancyGridPublisher : MonoBehaviour
{
    [Header("ROS")]
    [Tooltip("Topic name. Convention for a static map is /map.")]
    public string topicName = "map";

    [Tooltip("frame_id written into the message header.")]
    public string frameId = "map";

    [Tooltip("How often the map is re-sent. A static map is normally 'latched' (sent once and " +
             "remembered); re-sending at 1 Hz gives late-starting ROS nodes the same result.")]
    public float publishHz = 1f;

    [Header("Timing")]
    [Tooltip("Seconds to wait after Play before building the grid, so the warehouse has finished spawning.")]
    public float startDelay = 1f;

    [Tooltip("Rebuild the grid before every publish (only needed if shelves move at runtime).")]
    public bool rebuildEveryPublish = false;

    private ROSConnection ros;
    private CubeCarNavigator navigator;
    private OccupancyGridMsg msg;

    IEnumerator Start()
    {
        navigator = GetComponent<CubeCarNavigator>();
        if (navigator == null)
        {
            Debug.LogError("OccupancyGridPublisher: no CubeCarNavigator on this GameObject.", this);
            yield break;
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OccupancyGridMsg>(topicName);

        yield return new WaitForSeconds(startDelay);

        if (!BuildMessage()) yield break;

        WaitForSeconds wait = new WaitForSeconds(1f / Mathf.Max(0.1f, publishHz));
        while (true)
        {
            if (rebuildEveryPublish) BuildMessage();
            ros.Publish(topicName, msg);
            yield return wait;
        }
    }

    private bool BuildMessage()
    {
        if (!navigator.BuildGridForRos())
        {
            Debug.LogError("OccupancyGridPublisher: the navigator could not build a grid.", this);
            return false;
        }

        // Unity grid: ix runs along Unity X, iz along Unity Z.
        int uW = navigator.GridWidth;
        int uH = navigator.GridHeight;
        bool[] blocked = navigator.BlockedCells;
        float cs = navigator.cellSize;
        Vector2 o = navigator.GridOriginXZ; // (min X, min Z) in Unity world coordinates

        // ROS grid: i runs along ROS x (= Unity Z), j runs along ROS y (= -Unity X).
        int rosW = uH;
        int rosH = uW;
        sbyte[] data = new sbyte[rosW * rosH];

        for (int iz = 0; iz < uH; iz++)
        {
            for (int ix = 0; ix < uW; ix++)
            {
                int i = iz;            // ROS x grows with Unity Z
                int j = uW - 1 - ix;   // ROS y grows when Unity X shrinks -> flip
                data[j * rosW + i] = blocked[iz * uW + ix] ? (sbyte)100 : (sbyte)0;
            }
        }

        OccupancyGridMsg m = new OccupancyGridMsg();
        m.header = new HeaderMsg();
        m.header.frame_id = frameId;

        m.info.resolution = cs;
        m.info.width = (uint)rosW;
        m.info.height = (uint)rosH;

        // 'origin' = pose of the corner of cell (0, 0), i.e. the minimum-x, minimum-y corner.
        m.info.origin.position.x = o.y;               // min ROS x  = min Unity Z
        m.info.origin.position.y = -(o.x + uW * cs);  // min ROS y  = -(max Unity X)
        m.info.origin.position.z = 0.0;
        m.info.origin.orientation.w = 1.0;            // no rotation

        m.data = data;
        msg = m;

        Debug.Log("OccupancyGridPublisher: built map " + rosW + " x " + rosH + " cells, resolution " + cs +
                  " m, origin (" + m.info.origin.position.x + ", " + m.info.origin.position.y + ") in ROS coordinates.");
        return true;
    }
}