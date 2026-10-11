using System.Collections.Generic;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Reports every collision of the robot with a wall, shelf, box or NPC on /sim/collision (std_msgs/String, JSON:
/// other_tag, sim_time_s, x, y in the map frame, robot metres). The test plan counts these.
///
/// The robot is physical (ADR-010), so a collision is real contact. Contact with the floor is not a collision:
/// its normal points up. A second contact with the same object within one second is the same collision.
/// Also warns once in the Console if the robot tips over.
///
/// Put this on the robot's ROOT GameObject, next to DiffDriveController.
/// </summary>
[RequireComponent(typeof(DiffDriveController))]
[DisallowMultipleComponent]
public class CollisionReporter : MonoBehaviour
{
    public string topic = "/sim/collision";

    [Tooltip("A contact whose normal points up more than this is the floor.")]
    [Range(0f, 1f)]
    public float floorNormalY = 0.7f;

    [Tooltip("Repeat contacts with the same object within this time count once.")]
    public float debounceSeconds = 1f;

    [Tooltip("Warn once when the robot's up vector falls below this (it has tipped over).")]
    [Range(0f, 1f)]
    public float tipUpY = 0.7f;

    private ROSConnection ros;
    private DiffDriveController drive;
    private CollisionBook book;
    private bool tipped;
    private readonly List<Transform> chain = new List<Transform>();   // reused by TaggedObject
    private readonly List<string> chainTags = new List<string>();

    void Start()
    {
        drive = GetComponent<DiffDriveController>();
        book = new CollisionBook(debounceSeconds);
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(topic);

        // DiffDriveController rebuilt the contact colliders in Awake, so these are the final ones.
        foreach (Collider c in GetComponentsInChildren<Collider>(true)) Hook(c.gameObject);
        foreach (ArticulationBody b in GetComponentsInChildren<ArticulationBody>(true)) Hook(b.gameObject);
    }

    void Update()
    {
        if (tipped || drive.BaseBody.up.y >= tipUpY) return;
        tipped = true;
        Debug.LogWarning("CollisionReporter: the robot has tipped over (up.y=" + drive.BaseBody.up.y.ToString("F2") + ").", this);
    }

    private void Hook(GameObject go)
    {
        CollisionRelay relay = go.GetComponent<CollisionRelay>();
        if (relay == null) relay = go.AddComponent<CollisionRelay>();
        relay.reporter = this;
    }

    /// <summary>Called by <see cref="CollisionRelay"/> for every new contact of a robot link.</summary>
    public void OnRobotCollision(Collision collision)
    {
        Collider other = collision.collider;
        if (other == null || other.transform.IsChildOf(transform)) return; // one of our own links
        if (collision.contactCount == 0) return;

        ContactPoint contact = collision.GetContact(0);
        if (contact.normal.y > floorNormalY) return; // the floor under a wheel

        // The tag and the "same object" of the debounce belong to the tagged object, not to the leaf collider.
        Transform hit = TaggedObject(other);
        double now = Time.timeAsDouble;
        if (!book.ShouldReport(hit.GetInstanceID(), now)) return;

        Pose2D at = RosConversions.ToMapPose(contact.point, 0f, transform.lossyScale.x);
        string json = CollisionBook.ToJson(CollisionBook.OtherTag(hit.tag), now, at.X, at.Y);
        ros.Publish(topic, new StringMsg(json));
        Debug.LogWarning("CollisionReporter: " + json + " (" + hit.name + ")", this);
    }

    /// <summary>
    /// A rack is seven untagged colliders (legs, boards) under a root tagged "Shelf", and a robot can touch two of
    /// them in one bump. So walk from the hit collider up to the PhysicalBody that owns it and take the first
    /// tagged object on the way (a wall panel is tagged on the collider itself). Nothing tagged: the collider.
    /// </summary>
    private Transform TaggedObject(Collider other)
    {
        PhysicalBody owner = PhysicalBody.OwnerOf(other.transform);
        Transform stop = owner != null ? owner.transform : other.transform;
        chain.Clear();
        chainTags.Clear();
        for (Transform t = other.transform; t != null; t = t.parent)
        {
            chain.Add(t);
            chainTags.Add(t.tag);
            if (t == stop) break;
        }
        int found = CollisionBook.FirstTaggedIndex(chainTags);
        return found >= 0 ? chain[found] : other.transform;
    }
}
