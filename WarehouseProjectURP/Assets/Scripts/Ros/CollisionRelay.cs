using UnityEngine;

/// <summary>
/// Hands the collision callbacks of one robot link to the <see cref="CollisionReporter"/>. Unity delivers
/// OnCollisionEnter to the GameObject that owns the collider or the body, so the reporter adds one of these to
/// each of them at Start. Not added by hand.
/// </summary>
public sealed class CollisionRelay : MonoBehaviour
{
    public CollisionReporter reporter;

    void OnCollisionEnter(Collision collision)
    {
        if (reporter != null) reporter.OnRobotCollision(collision);
    }
}
