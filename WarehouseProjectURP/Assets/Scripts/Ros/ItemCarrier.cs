using System;
using System.Collections;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Visually manages the complete lifecycle of cargo:
/// 1. Pre-places cargo on target shelf at startup (Robot starts EMPTY).
/// 2. On /sim/attach_item: Animates smooth PICK UP from shelf onto robot bed (drive.BaseBody).
///    CRITICAL FIX: Disables cargo collider & removes Rigidbody while carried to prevent PhysX penetration explosion/tipping over!
/// 3. On /sim/release_item: Animates smooth DROP OFF from robot onto delivery pallet.
/// </summary>
[DisallowMultipleComponent]
public class ItemCarrier : MonoBehaviour
{
    public string attachTopic = "/sim/attach_item";
    public string releaseTopic = "/sim/release_item";
    public string ackTopic = "/sim/ack";

    [Tooltip("Target parent transform on the robot. Defaults to drive.BaseBody (base_link).")]
    public Transform attachParent;

    [Tooltip("Local offset when carried on the robot body.")]
    public Vector3 localAttachOffset = new Vector3(0f, 0.22f, -0.05f);

    [Tooltip("Target shelf item location in Unity coordinates (ROS x=-3.75, y=-3.13).")]
    public Vector3 shelfItemPosition = new Vector3(12.52f, 1.20f, -15.00f);

    [Tooltip("Drop-off station location in Unity coordinates (ROS HomePoint x=6.32, y=5.87).")]
    public Vector3 dropoffStationPosition = new Vector3(-23.48f, 0.40f, 25.28f);

    private ROSConnection ros;
    private DiffDriveController drive;
    private GameObject currentCarriedItem;
    private GameObject designatedCargoBox;

    public Transform GetAttachTransform()
    {
        if (attachParent != null) return attachParent;
        if (drive == null) drive = GetComponent<DiffDriveController>();
        if (drive != null && drive.BaseBody != null) return drive.BaseBody;

        Transform baseLink = transform.Find("base_footprint/base_link") ?? transform.Find("base_link");
        if (baseLink != null) return baseLink;

        return transform;
    }

    void Start()
    {
        drive = GetComponent<DiffDriveController>();
        attachParent = GetAttachTransform();

        ros = ROSConnection.GetOrCreateInstance();
        try { ros.RegisterPublisher<StringMsg>(ackTopic); } catch {}
        ros.Subscribe<Int32Msg>(attachTopic, OnAttachItem);
        ros.Subscribe<EmptyMsg>(releaseTopic, OnReleaseItem);

        InitializeShelfCargo();
    }

    void OnDestroy()
    {
        if (ros != null)
        {
            ros.Unsubscribe(attachTopic);
            ros.Unsubscribe(releaseTopic);
        }
    }

    private void InitializeShelfCargo()
    {
        designatedCargoBox = GameObject.Find("WarehouseCargoBox_001");

        if (designatedCargoBox == null)
        {
            designatedCargoBox = GameObject.CreatePrimitive(PrimitiveType.Cube);
            designatedCargoBox.name = "WarehouseCargoBox_001";
        }

        // NON-STATIC
        designatedCargoBox.isStatic = false;

        // Apply distinct glowing orange/gold cargo material
        Renderer rend = designatedCargoBox.GetComponent<Renderer>();
        if (rend != null)
        {
            Shader shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
            if (shader != null)
            {
                Material cargoMat = new Material(shader);
                cargoMat.color = new Color(1.0f, 0.55f, 0.05f); // Vibrant Cargo Orange
                rend.material = cargoMat;
            }
        }

        // Pre-place box on shelf: ROBOT STARTS EMPTY!
        designatedCargoBox.transform.SetParent(null, true);
        designatedCargoBox.transform.localScale = new Vector3(1.2f, 0.9f, 1.2f);
        designatedCargoBox.transform.position = shelfItemPosition;
        designatedCargoBox.transform.rotation = Quaternion.identity;

        // Enable trigger/kinematic on shelf
        var boxCol = designatedCargoBox.GetComponent<Collider>();
        if (boxCol != null) boxCol.enabled = true;

        var boxRb = designatedCargoBox.GetComponent<Rigidbody>();
        if (boxRb == null) boxRb = designatedCargoBox.AddComponent<Rigidbody>();
        boxRb.isKinematic = true;
        boxRb.useGravity = false;

        Debug.Log("ItemCarrier: WarehouseCargoBox_001 positioned on Target Shelf. Robot is EMPTY and ready for mission.");
    }

    private void OnAttachItem(Int32Msg msg)
    {
        try
        {
            if (designatedCargoBox == null)
            {
                designatedCargoBox = GameObject.Find("WarehouseCargoBox_001");
            }

            if (designatedCargoBox != null)
            {
                StartCoroutine(PickUpItemRoutine(designatedCargoBox));
            }
            else
            {
                SendAck("attach_item", false, "WarehouseCargoBox_001 not found");
            }
        }
        catch (Exception ex)
        {
            SendAck("attach_item", false, ex.Message);
        }
    }

    private void OnReleaseItem(EmptyMsg msg)
    {
        try
        {
            Transform movingBody = GetAttachTransform();
            if (currentCarriedItem != null)
            {
                StartCoroutine(DropOffItemRoutine(currentCarriedItem));
            }
            else if (designatedCargoBox != null && designatedCargoBox.transform.IsChildOf(movingBody))
            {
                StartCoroutine(DropOffItemRoutine(designatedCargoBox));
            }
            else
            {
                SendAck("release_item", true, "No item was carried");
            }
        }
        catch (Exception ex)
        {
            SendAck("release_item", false, ex.Message);
        }
    }

    private IEnumerator PickUpItemRoutine(GameObject item)
    {
        currentCarriedItem = item;
        Transform movingBody = GetAttachTransform();

        // CRITICAL FIX: Disable collider and remove Rigidbody IMMEDIATELY before parenting!
        // Prevents ArticulationBody self-collision penetration explosion / tipping over!
        Collider itemCol = item.GetComponent<Collider>();
        if (itemCol != null) itemCol.enabled = false;

        Rigidbody itemRb = item.GetComponent<Rigidbody>();
        if (itemRb != null) Destroy(itemRb);

        Vector3 startWorld = item.transform.position;
        float elapsed = 0f;
        float duration = 0.6f;

        // Parent to moving body right away so coordinate systems move together
        item.transform.SetParent(movingBody, true);

        while (elapsed < duration)
        {
            elapsed += Time.deltaTime;
            float t = Mathf.Clamp01(elapsed / duration);
            float smoothT = Mathf.SmoothStep(0f, 1f, t);

            Vector3 targetWorld = movingBody.TransformPoint(localAttachOffset);
            Vector3 currentPos = Vector3.Lerp(startWorld, targetWorld, smoothT);
            currentPos.y += Mathf.Sin(smoothT * Mathf.PI) * 0.35f; // Gentle lift arc

            item.transform.position = currentPos;
            yield return null;
        }

        // Lock firmly into local position on the robot bed
        item.transform.localPosition = localAttachOffset;
        item.transform.localRotation = Quaternion.identity;
        // In base_link (robot scale 4), local scale (0.30, 0.225, 0.30) translates to (1.2, 0.9, 1.2) world size
        item.transform.localScale = new Vector3(0.30f, 0.225f, 0.30f);

        Debug.Log("[PICK UP SUCCESS] Cargo item safely mounted on robot bed without physics interference.");
        SendAck("attach_item", true, "");
    }

    private IEnumerator DropOffItemRoutine(GameObject item)
    {
        Vector3 startWorld = item.transform.position;
        // Deliver right in front of the robot or onto dropoffStationPosition
        Transform movingBody = GetAttachTransform();
        Vector3 targetWorld = (movingBody != null) ? movingBody.position + movingBody.forward * 1.5f : dropoffStationPosition;
        targetWorld.y = 0.40f; // Sits neatly on warehouse floor/station pallet

        float elapsed = 0f;
        float duration = 0.6f;

        item.transform.SetParent(null, true);

        while (elapsed < duration)
        {
            elapsed += Time.deltaTime;
            float t = Mathf.Clamp01(elapsed / duration);
            float smoothT = Mathf.SmoothStep(0f, 1f, t);

            Vector3 currentPos = Vector3.Lerp(startWorld, targetWorld, smoothT);
            currentPos.y += Mathf.Sin(smoothT * Mathf.PI) * 0.25f;

            item.transform.position = currentPos;
            yield return null;
        }

        item.transform.position = targetWorld;
        item.transform.rotation = Quaternion.identity;
        item.transform.localScale = new Vector3(1.2f, 0.9f, 1.2f);

        // Re-enable collider and kinematic Rigidbody now that it rests stably on the pallet
        var rb = item.GetComponent<Rigidbody>();
        if (rb == null) rb = item.AddComponent<Rigidbody>();
        rb.isKinematic = true;
        rb.useGravity = false;

        var col = item.GetComponent<Collider>();
        if (col != null) col.enabled = true;

        currentCarriedItem = null;
        Debug.Log("[DROP OFF SUCCESS] Cargo item safely delivered to station. Robot is EMPTY.");
        SendAck("release_item", true, "");
    }

    private void SendAck(string cmd, bool ok, string error)
    {
        string json = $"{{\"cmd\":\"{cmd}\",\"ok\":{(ok ? "true" : "false")},\"error\":\"{error}\"}}";
        try
        {
            ros.Publish(ackTopic, new StringMsg(json));
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"SendAck error: {ex.Message}");
        }
    }
}
