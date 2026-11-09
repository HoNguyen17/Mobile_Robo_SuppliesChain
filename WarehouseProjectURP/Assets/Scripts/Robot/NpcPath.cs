using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Where a scripted worker is, as a function of the time since it started (ADR-019). Pure: no GameObject, no clock, so it
/// is unit-tested. Positions are ROS metres. The same maths is in scenario_spec.py (position_at, seeded_unit) and the test
/// vectors are the same.
///
/// A worker waits dwell_s at each waypoint, then walks to the next at speed_mps. "once" stops at the last waypoint for
/// good; "pingpong" walks back along the waypoints and repeats.
/// </summary>
public sealed class NpcPath
{
    private readonly List<NpcWaypoint> sequence;
    private readonly float speed;
    private readonly bool closed;

    public NpcPath(NpcWorkerSpec spec)
    {
        if (spec == null || spec.waypoints == null || spec.waypoints.Count < 2)
            throw new ArgumentException("a worker needs at least 2 waypoints");
        if (!(spec.speed_mps > 0f)) throw new ArgumentException("speed_mps must be > 0");
        if (spec.mode != "once" && spec.mode != "pingpong") throw new ArgumentException("mode must be once or pingpong");

        speed = spec.speed_mps;
        closed = spec.mode == "pingpong";
        sequence = new List<NpcWaypoint>(spec.waypoints);
        if (closed)
        {
            for (int i = spec.waypoints.Count - 2; i >= 1; i--) sequence.Add(spec.waypoints[i]);
        }
    }

    /// <summary>Seconds for one pass ("once") or one cycle ("pingpong"); infinity when the worker waits for ever.</summary>
    public float Duration()
    {
        float total = 0f;
        for (int i = 0; i < sequence.Count; i++)
        {
            if (!closed && i == sequence.Count - 1) break;
            if (sequence[i].dwell_s < 0f) return float.PositiveInfinity;
            total += sequence[i].dwell_s + LegSeconds(i);
        }
        return total;
    }

    /// <summary>Position in ROS metres <paramref name="t"/> seconds after the worker started.</summary>
    public Vector2 PositionAt(float t)
    {
        t = Mathf.Max(0f, t);
        if (closed)
        {
            float period = Duration();
            if (period > 0f && !float.IsInfinity(period)) t = t % period;
        }
        for (int i = 0; i < sequence.Count; i++)
        {
            NpcWaypoint wp = sequence[i];
            if (!closed && i == sequence.Count - 1) return Point(wp);
            if (wp.dwell_s < 0f || t < wp.dwell_s) return Point(wp);
            t -= wp.dwell_s;

            float leg = LegSeconds(i);
            if (t < leg)
            {
                NpcWaypoint next = sequence[(i + 1) % sequence.Count];
                float k = leg > 0f ? t / leg : 1f;
                return Vector2.Lerp(Point(wp), Point(next), k);
            }
            t -= leg;
        }
        return Point(sequence[0]);
    }

    /// <summary>Unit direction of travel at time <paramref name="t"/>, or zero while the worker stands still.</summary>
    public Vector2 HeadingAt(float t)
    {
        Vector2 delta = PositionAt(t + 0.05f) - PositionAt(t);
        return delta.sqrMagnitude > 1e-8f ? delta.normalized : Vector2.zero;
    }

    private float LegSeconds(int i)
    {
        Vector2 a = Point(sequence[i]);
        Vector2 b = Point(sequence[(i + 1) % sequence.Count]);
        return (b - a).magnitude / speed;
    }

    private static Vector2 Point(NpcWaypoint wp)
    {
        return new Vector2(wp.x, wp.y);
    }

    /// <summary>Deterministic value in [0, 1) from (seed, salt): a 32-bit integer hash, identical to scenario_spec.seeded_unit.</summary>
    public static double SeededUnit(int seed, int salt)
    {
        unchecked
        {
            uint x = (uint)seed * 0x9E3779B1u + (uint)salt * 0x85EBCA6Bu + 0x165667B1u;
            x ^= x >> 15;
            x *= 0x2C1B3C6Du;
            x ^= x >> 12;
            x *= 0x297A2D39u;
            x ^= x >> 15;
            return x / 4294967296.0;
        }
    }

    /// <summary>Seconds from the trigger (reset, or the robot leaving its start pose) until the worker appears.</summary>
    public static double StartDelay(NpcWorkerSpec worker, int seed, int workerIndex)
    {
        return worker.start_delay_s + worker.start_jitter_s * SeededUnit(seed, workerIndex);
    }
}
