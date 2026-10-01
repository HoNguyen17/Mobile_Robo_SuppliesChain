using System.Collections.Generic;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

/// <summary>
/// Drives a URDF-imported differential-drive robot (TurtleBot3 Waffle Pi) through its
/// wheel ArticulationBodies. It knows nothing about planning: something else calls
/// <see cref="SetCommand"/> with a velocity, exactly like a ROS /cmd_vel message.
///
/// Conventions follow ROS (REP-103), so the same numbers work for /cmd_vel later:
///   linear  = ROBOT metres per second, positive = forward
///   angular = radians per second, positive = counter-clockwise seen from above (turn left)
///
/// The robot may be scaled up in the scene (ADR-010). All lengths and speeds here are in ROBOT
/// metres and are multiplied by the root's scale where they meet the world.
///
/// Put this on the robot's ROOT GameObject (e.g. "turtlebot3_waffle_pi").
/// </summary>
[DisallowMultipleComponent]
public class DiffDriveController : MonoBehaviour
{
    [Header("Wheels (found by name if left empty)")]
    public ArticulationBody leftWheel;
    public ArticulationBody rightWheel;
    public string leftWheelName = "wheel_left_link";
    public string rightWheelName = "wheel_right_link";

    [Header("Geometry (robot metres, from turtlebot3_waffle_pi.urdf)")]
    [Tooltip("Wheel radius.")]
    public float wheelRadius = 0.033f;

    [Tooltip("Distance between the two wheel centres (2 x 0.144).")]
    public float wheelSeparation = 0.288f;

    [Header("Speed limits (TurtleBot3 Waffle Pi)")]
    public float maxLinearSpeed = 0.26f;    // m/s
    public float maxAngularSpeed = 1.82f;   // rad/s

    [Tooltip("Speed ramps toward the command at this rate, so the robot never jerks or wheelies. m/s^2 (robot metres).")]
    public float maxLinearAccel = 1.0f;

    [Tooltip("rad/s^2.")]
    public float maxAngularAccel = 3.0f;

    [Header("Wheel motors")]
    [Tooltip("Peak torque per wheel in N*m at robot scale 1. It is multiplied by scale^5 " +
             "(mass ~ s^3, acceleration ~ s, lever arm ~ s), matching the mass scaling below.")]
    public float wheelTorqueLimit = 2f;

    [Tooltip("Tick if this wheel spins the wrong way (robot drives backwards or spins in place " +
             "when told to go forward). Test with 'Keyboard Teleop'.")]
    public bool invertLeftWheel;
    public bool invertRightWheel;

    [Tooltip("Lock the wheel positions once the robot is commanded to stop and has ramped down, like the " +
             "holding torque of the real Dynamixel motors. Without it the robot slowly creeps when idle.")]
    public bool holdWhenStopped = true;

    [Tooltip("The holding brake reaches full motor torque at this many degrees of wheel rotation.")]
    public float holdFullTorqueDeg = 2f;

    [Header("Contacts (the URDF import's colliders float / dig in, so they are rebuilt at start)")]
    [Tooltip("Replace wheel colliders with spheres, casters with frictionless spheres at wheel-bottom height, " +
             "and lift the chassis clear of the floor. Untick only for debugging.")]
    public bool rebuildContacts = true;

    [Tooltip("Rubber grip of the wheels.")]
    public float wheelGrip = 1.5f;

    [Tooltip("Caster ball radius (robot metres).")]
    public float casterRadius = 0.012f;

    [Tooltip("Chassis colliders must stay at least this far above the wheel bottoms (robot metres).")]
    public float chassisClearance = 0.02f;

    public string casterNameContains = "caster";

    [Header("Safety")]
    [Tooltip("Stop the wheels if no command arrives for this many seconds (0 = never). " +
             "Mirrors the 0.5 s /cmd_vel watchdog in docs/architecture.md.")]
    public float commandTimeout = 0.5f;

    [Header("Debug")]
    [Tooltip("Drive with the arrow keys (Up/Down = forward/back, Left/Right = turn). " +
             "Use this to check the wheel directions.")]
    public bool keyboardTeleop;

    /// <summary>Last accepted linear command (m/s), after clamping.</summary>
    public float CommandedLinear { get; private set; }

    /// <summary>Last accepted angular command (rad/s), after clamping.</summary>
    public float CommandedAngular { get; private set; }

    /// <summary>
    /// The part of the robot that actually moves: the articulation root (for the URDF import that is
    /// 'base_link'). The GameObjects above it (root, base_footprint) stay where they were placed,
    /// so anything that needs the robot's position must read this transform.
    /// </summary>
    public Transform BaseBody
    {
        get
        {
            if (baseBody == null) baseBody = FindArticulationRoot();
            return baseBody;
        }
    }

    private Transform baseBody;
    private float lastCommandTime = float.NegativeInfinity;
    private float currentLinear;
    private float currentAngular;
    private float holdStiffness;
    private bool holding;

    /// <summary>True while the holding brake locks the wheels.</summary>
    public bool IsHolding => holding;

    void Awake()
    {
        if (leftWheel == null) leftWheel = FindBody(leftWheelName);
        if (rightWheel == null) rightWheel = FindBody(rightWheelName);

        if (leftWheel == null || rightWheel == null)
        {
            Debug.LogError("DiffDriveController: could not find the wheel ArticulationBodies '" +
                           leftWheelName + "' / '" + rightWheelName + "'. Assign them in the Inspector.", this);
            enabled = false;
            return;
        }

        FloorColliderFix.Apply(); // zero-thickness warehouse floors let robot parts sink through

        float scale = transform.lossyScale.x;
        ConfigureMotor(leftWheel, scale);
        ConfigureMotor(rightWheel, scale);
        MakeRootMovable();
        ScaleMassProperties(scale);
        if (rebuildContacts) BuildContacts(scale);
    }

    // ------------------------------------------------------------------
    // Public API
    // ------------------------------------------------------------------

    /// <summary>Set the robot's velocity (ROS conventions). Must be refreshed within 'commandTimeout'.</summary>
    public void SetCommand(float linear, float angular)
    {
        CommandedLinear = Mathf.Clamp(linear, -maxLinearSpeed, maxLinearSpeed);
        CommandedAngular = Mathf.Clamp(angular, -maxAngularSpeed, maxAngularSpeed);
        lastCommandTime = Time.time;
    }

    /// <summary>Command zero velocity.</summary>
    public void Stop()
    {
        SetCommand(0f, 0f);
    }

    // ------------------------------------------------------------------

    void FixedUpdate()
    {
        if (keyboardTeleop) ReadKeyboard();

        float v = CommandedLinear;
        float w = CommandedAngular;
        if (commandTimeout > 0f && Time.time - lastCommandTime > commandTimeout)
        {
            v = 0f;
            w = 0f;
        }

        float dt = Time.fixedDeltaTime;
        currentLinear = Mathf.MoveTowards(currentLinear, v, maxLinearAccel * dt);
        currentAngular = Mathf.MoveTowards(currentAngular, w, maxAngularAccel * dt);

        bool hold = holdWhenStopped && WheelHold.ShouldHold(v, w, currentLinear, currentAngular);
        if (hold != holding)
        {
            holding = hold;
            SetHold(leftWheel, hold);
            SetHold(rightWheel, hold);
        }
        if (holding) return;

        // Differential-drive kinematics: a left turn (w > 0) makes the right wheel faster.
        float leftRadPerSec = (currentLinear - currentAngular * wheelSeparation * 0.5f) / wheelRadius;
        float rightRadPerSec = (currentLinear + currentAngular * wheelSeparation * 0.5f) / wheelRadius;

        SetWheelSpeed(leftWheel, leftRadPerSec, invertLeftWheel);
        SetWheelSpeed(rightWheel, rightRadPerSec, invertRightWheel);
    }

    private void ReadKeyboard()
    {
        float forward = 0f;
        float turn = 0f;
        if (Input.GetKey(KeyCode.UpArrow)) forward += 1f;
        if (Input.GetKey(KeyCode.DownArrow)) forward -= 1f;
        if (Input.GetKey(KeyCode.LeftArrow)) turn += 1f;   // left = counter-clockwise = positive
        if (Input.GetKey(KeyCode.RightArrow)) turn -= 1f;

        if (forward != 0f || turn != 0f)
        {
            SetCommand(forward * maxLinearSpeed, turn * maxAngularSpeed);
        }
    }

    /// <summary>Lock the wheel at its current angle (position spring), or release it back to velocity control.</summary>
    private void SetHold(ArticulationBody wheel, bool on)
    {
        ArticulationDrive drive = wheel.xDrive;
        drive.targetVelocity = 0f;
        if (on)
        {
            drive.stiffness = holdStiffness;
            drive.target = wheel.jointPosition[0] * Mathf.Rad2Deg; // jointPosition is radians, the drive wants degrees
        }
        else
        {
            drive.stiffness = 0f;
        }
        wheel.xDrive = drive;
    }

    private static void SetWheelSpeed(ArticulationBody wheel, float radPerSec, bool invert)
    {
        ArticulationDrive drive = wheel.xDrive;
        drive.targetVelocity = (invert ? -radPerSec : radPerSec) * Mathf.Rad2Deg; // Unity wants deg/s
        wheel.xDrive = drive;
    }

    // ------------------------------------------------------------------
    // Setup helpers
    // ------------------------------------------------------------------

    private Transform FindArticulationRoot()
    {
        foreach (ArticulationBody body in GetComponentsInChildren<ArticulationBody>())
        {
            Transform parent = body.transform.parent;
            if (parent == null || parent.GetComponentInParent<ArticulationBody>() == null) return body.transform;
        }
        return transform;
    }

    private ArticulationBody FindBody(string linkName)
    {
        foreach (ArticulationBody body in GetComponentsInChildren<ArticulationBody>())
        {
            if (body.name == linkName) return body;
        }
        return null;
    }

    private void ConfigureMotor(ArticulationBody wheel, float scale)
    {
        // Velocity control: no spring (stiffness 0). Torque = damping x speed error, capped at forceLimit.
        // Damping is chosen so the cap is reached at ~50 deg/s of error, i.e. a firm but smooth motor.
        float torqueLimit = wheelTorqueLimit * Mathf.Pow(scale, 5f);
        holdStiffness = WheelHold.Stiffness(torqueLimit, holdFullTorqueDeg);
        ArticulationDrive drive = wheel.xDrive;
        drive.stiffness = 0f;
        drive.damping = torqueLimit / 50f;
        drive.forceLimit = torqueLimit;
        drive.targetVelocity = 0f;
        wheel.xDrive = drive;
        wheel.maxAngularVelocity = 100f; // the importer default (7 rad/s) would cap the robot at ~0.23 m/s
    }

    private void MakeRootMovable()
    {
        ArticulationBody rootBody = BaseBody.GetComponent<ArticulationBody>();
        if (rootBody != null && rootBody.immovable)
        {
            rootBody.immovable = false;
            Debug.Log("DiffDriveController: '" + rootBody.name + "' was Immovable; switched it off so the robot can drive.", this);
        }
    }

    /// <summary>
    /// The URDF import leaves the robot resting on its chassis/caster boxes with the wheels hovering
    /// (scale 1) or dug into the floor (scale 4), so the wheels have no traction. Rebuild the
    /// floor contacts: wheel spheres that carry the load, frictionless caster balls at wheel-bottom
    /// height, and slick chassis colliders lifted clear of the floor.
    /// </summary>
    private void BuildContacts(float scale)
    {
        // Unity picks the combine mode by priority Maximum > Multiply > Average > Minimum, and the floor uses
        // the default (Average). So "Minimum" would lose to it; "Multiply" with 0 friction gives exactly 0.
        Physics.SyncTransforms();

        PhysicMaterial grip = new PhysicMaterial("WheelGrip")
        {
            staticFriction = wheelGrip,
            dynamicFriction = wheelGrip,
            frictionCombine = PhysicMaterialCombine.Maximum,
            bounceCombine = PhysicMaterialCombine.Minimum
        };
        PhysicMaterial slick = new PhysicMaterial("BodySlick")
        {
            staticFriction = 0f,
            dynamicFriction = 0f,
            frictionCombine = PhysicMaterialCombine.Multiply,
            bounceCombine = PhysicMaterialCombine.Minimum
        };

        HashSet<Collider> mine = new HashSet<Collider>();
        float wheelBottom = float.MaxValue;

        // 1. Wheels: a sphere at the wheel's own centre.
        foreach (ArticulationBody wheel in new[] { leftWheel, rightWheel })
        {
            Vector3 centreWorld = wheel.transform.position;
            Bounds old = default;
            bool hasBounds = false;
            foreach (Collider c in wheel.GetComponentsInChildren<Collider>())
            {
                if (!hasBounds) { old = c.bounds; hasBounds = true; } else old.Encapsulate(c.bounds);
            }
            if (hasBounds) centreWorld = old.center;

            RemoveColliders(wheel.transform);
            SphereCollider sphere = wheel.gameObject.AddComponent<SphereCollider>();
            sphere.radius = wheelRadius;
            sphere.center = wheel.transform.InverseTransformPoint(centreWorld);
            sphere.sharedMaterial = grip;
            mine.Add(sphere);
            wheelBottom = Mathf.Min(wheelBottom, centreWorld.y - wheelRadius * scale);
        }

        // 2. Casters: frictionless balls whose bottoms sit at the wheel-bottom plane.
        foreach (ArticulationBody body in GetComponentsInChildren<ArticulationBody>())
        {
            if (!body.name.Contains(casterNameContains)) continue;
            RemoveColliders(body.transform);
            Vector3 p = body.transform.position;
            Vector3 centreWorld = new Vector3(p.x, wheelBottom + casterRadius * scale, p.z);
            SphereCollider ball = body.gameObject.AddComponent<SphereCollider>();
            ball.radius = casterRadius;
            ball.center = body.transform.InverseTransformPoint(centreWorld);
            ball.sharedMaterial = slick;
            mine.Add(ball);
        }

        // 3. Everything else: slick, and never lower than the clearance plane.
        float floorPlane = wheelBottom + chassisClearance * scale;
        foreach (Collider c in GetComponentsInChildren<Collider>())
        {
            if (mine.Contains(c) || !c.enabled || c.isTrigger) continue;
            c.sharedMaterial = slick;
            float lowest = c.bounds.min.y;
            if (lowest >= floorPlane) continue;

            BoxCollider box = c as BoxCollider;
            if (box != null)
            {
                float lift = floorPlane - lowest;
                box.center += box.transform.InverseTransformVector(Vector3.up * lift);
            }
            else
            {
                c.enabled = false;
                Debug.LogWarning("DiffDriveController: disabled low collider '" + c.name + "' on '" +
                                 c.transform.parent.name + "' (not a box, cannot be lifted).", this);
            }
        }

        IgnoreSelfCollisions();
    }

    /// <summary>The robot's own parts must never collide with each other (wheels vs chassis etc.).</summary>
    private void IgnoreSelfCollisions()
    {
        List<Collider> parts = new List<Collider>();
        foreach (Collider c in GetComponentsInChildren<Collider>())
        {
            if (c.enabled && !c.isTrigger) parts.Add(c);
        }
        for (int i = 0; i < parts.Count; i++)
        {
            for (int j = i + 1; j < parts.Count; j++)
            {
                Physics.IgnoreCollision(parts[i], parts[j], true);
            }
        }
    }

    /// <summary>
    /// Unity scales colliders with the transform but not mass, centre of mass or inertia. For a robot scaled
    /// by s make them consistent with the geometry: mass x s^3, centre of mass x s, inertia x s^5.
    /// Runs in Awake, i.e. before the URDF importer applies its stored inertia data in Start.
    /// </summary>
    private void ScaleMassProperties(float scale)
    {
        // Links that have no <inertial> in the URDF (camera, imu ...) keep Unity's default mass of 1 kg each.
        // That phantom 3-4 kg sits ahead of the axle and tips the 1.4 kg robot forward, so make them weightless.
        foreach (ArticulationBody body in GetComponentsInChildren<ArticulationBody>())
        {
            UrdfInertial data = body.GetComponent<UrdfInertial>();
            if (data == null || !data.useUrdfData) body.mass = 0.001f;
        }

        if (Mathf.Abs(scale - 1f) < 0.001f) return;

        float s3 = scale * scale * scale;
        float s5 = s3 * scale * scale;
        foreach (ArticulationBody body in GetComponentsInChildren<ArticulationBody>())
        {
            body.mass *= s3;
        }
        // The importer keeps the URDF centre of mass / inertia in UrdfInertial and applies them in Start().
        foreach (UrdfInertial inertial in GetComponentsInChildren<UrdfInertial>())
        {
            if (!inertial.useUrdfData) continue;
            inertial.centerOfMass *= scale;
            inertial.inertiaTensor *= s5;
        }
        Debug.Log("DiffDriveController: scaled masses x" + s3 + " and inertia x" + s5 + " for robot scale " + scale + ".", this);
    }

    private static void RemoveColliders(Transform link)
    {
        foreach (Collider c in link.GetComponentsInChildren<Collider>())
        {
            c.enabled = false; // leaves the physics world immediately; Destroy is deferred
            Destroy(c);
        }
    }
}
