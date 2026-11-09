using System.Collections.Generic;
using System.IO;
using NUnit.Framework;
using UnityEngine;

/// <summary>
/// The worker path maths, the seeded offset and the reset message of ADR-019. The vectors are the same as in
/// ros1/warehouse_eval/test/test_scenario_spec.py, so Python and C# cannot drift apart.
/// </summary>
public class NpcPathTests
{
    private static NpcWorkerSpec Worker(string mode, params float[][] points)
    {
        var spec = new NpcWorkerSpec
        {
            name = "w", speed_mps = 0.5f, radius_m = 0.15f, mode = mode, start_trigger = "robot_departs",
            waypoints = new List<NpcWaypoint>()
        };
        foreach (float[] p in points)
        {
            spec.waypoints.Add(new NpcWaypoint { x = p[0], y = p[1], dwell_s = p.Length > 2 ? p[2] : 0f });
        }
        return spec;
    }

    [Test]
    public void Seeded_values_match_the_python_vectors()
    {
        Assert.AreEqual(0.330931, NpcPath.SeededUnit(0, 0), 1e-6);
        Assert.AreEqual(0.27441, NpcPath.SeededUnit(7, 0), 1e-6);
        Assert.AreEqual(0.352948, NpcPath.SeededUnit(7, 1), 1e-6);
        Assert.AreEqual(0.479139, NpcPath.SeededUnit(1, 0), 1e-6);
        Assert.AreEqual(0.875656, NpcPath.SeededUnit(int.MaxValue, 5), 1e-6);
    }

    [Test]
    public void Start_delay_is_the_base_plus_the_seeded_share_of_the_jitter()
    {
        NpcWorkerSpec w = Worker("once", new[] { 0f, 0f }, new[] { 1f, 0f });
        w.start_delay_s = 2f;
        w.start_jitter_s = 4f;
        Assert.AreEqual(2.0 + 4.0 * 0.27441, NpcPath.StartDelay(w, 7, 0), 1e-5);
        w.start_jitter_s = 0f;
        Assert.AreEqual(2.0, NpcPath.StartDelay(w, 99, 3), 1e-9);
    }

    [Test]
    public void A_once_worker_walks_then_holds_the_last_waypoint()
    {
        var path = new NpcPath(Worker("once", new[] { 0f, 0f }, new[] { 3f, 0f }));
        Assert.AreEqual(6f, path.Duration(), 1e-4);
        Assert.AreEqual(0f, path.PositionAt(0f).x, 1e-4);
        Assert.AreEqual(1.5f, path.PositionAt(3f).x, 1e-4);
        Assert.AreEqual(3f, path.PositionAt(6f).x, 1e-4);
        Assert.AreEqual(3f, path.PositionAt(600f).x, 1e-4);
        Assert.AreEqual(0f, path.PositionAt(-5f).x, 1e-4);
    }

    [Test]
    public void A_dwell_holds_the_worker_then_it_walks_on()
    {
        var path = new NpcPath(Worker("once", new[] { 0f, 0f }, new[] { 3f, 0f, 4f }, new[] { 3f, 3f }));
        Assert.AreEqual(0f, path.PositionAt(9.99f).y, 1e-4);
        Assert.AreEqual(1.5f, path.PositionAt(13f).y, 1e-4);
        Assert.AreEqual(3f, path.PositionAt(16f).y, 1e-4);
    }

    [Test]
    public void A_forever_dwell_never_leaves()
    {
        var path = new NpcPath(Worker("once", new[] { 0f, 0f }, new[] { 3f, 0f, -1f }, new[] { 3f, 3f }));
        Assert.AreEqual(3f, path.PositionAt(1e6f).x, 1e-4);
        Assert.AreEqual(0f, path.PositionAt(1e6f).y, 1e-4);
        Assert.IsTrue(float.IsInfinity(path.Duration()));
    }

    [Test]
    public void Pingpong_goes_there_and_back()
    {
        var path = new NpcPath(Worker("pingpong", new[] { 0f, 0f }, new[] { 3f, 0f }));
        Assert.AreEqual(12f, path.Duration(), 1e-4);
        Assert.AreEqual(1.5f, path.PositionAt(9f).x, 1e-4);
        Assert.AreEqual(0f, path.PositionAt(12f).x, 1e-4);
        Assert.AreEqual(1.5f, path.PositionAt(15f).x, 1e-4);
    }

    [Test]
    public void Pingpong_with_three_waypoints_revisits_the_middle_one()
    {
        var path = new NpcPath(Worker("pingpong", new[] { 0f, 0f }, new[] { 1f, 0f }, new[] { 1f, 1f }));
        Assert.AreEqual(8f, path.Duration(), 1e-4);
        Assert.AreEqual(1f, path.PositionAt(4f).y, 1e-4);
        Assert.AreEqual(0f, path.PositionAt(6f).y, 1e-4);
        Assert.AreEqual(0.5f, path.PositionAt(7f).x, 1e-4);
    }

    [Test]
    public void Heading_points_along_the_leg_and_is_zero_when_the_worker_stands()
    {
        var path = new NpcPath(Worker("once", new[] { 0f, 0f }, new[] { 3f, 0f }));
        Assert.AreEqual(1f, path.HeadingAt(1f).x, 1e-4);
        Assert.AreEqual(0f, path.HeadingAt(100f).magnitude, 1e-4);
    }

    [Test]
    public void A_bad_worker_is_refused()
    {
        Assert.Throws<System.ArgumentException>(() => new NpcPath(Worker("loop", new[] { 0f, 0f }, new[] { 1f, 0f })));
        Assert.Throws<System.ArgumentException>(() => new NpcPath(Worker("once", new[] { 0f, 0f })));
        NpcWorkerSpec still = Worker("once", new[] { 0f, 0f }, new[] { 1f, 0f });
        still.speed_mps = 0f;
        Assert.Throws<System.ArgumentException>(() => new NpcPath(still));
    }

    [Test]
    public void A_plain_id_has_no_seed()
    {
        string id; bool hasSeed; int seed; string error;
        Assert.IsTrue(ResetRequest.TryParse("  S-04\n", out id, out hasSeed, out seed, out error));
        Assert.AreEqual("S-04", id);
        Assert.IsFalse(hasSeed);
    }

    [Test]
    public void A_seeded_id_is_split()
    {
        string id; bool hasSeed; int seed; string error;
        Assert.IsTrue(ResetRequest.TryParse("S-06?seed=7", out id, out hasSeed, out seed, out error));
        Assert.AreEqual("S-06", id);
        Assert.IsTrue(hasSeed);
        Assert.AreEqual(7, seed);
        Assert.IsTrue(ResetRequest.TryParse("S-00c?seed=0", out id, out hasSeed, out seed, out error));
        Assert.AreEqual(0, seed);
    }

    [Test]
    public void Malformed_reset_messages_are_refused_with_a_reason()
    {
        string[] bad = { "S-04?seed=-1", "S-04?seed=", "S-04?seed=x", "?seed=3", "S-04?x=1", "S-04?seed=1&seed=2", "S-04?", "S-04?seed=1234567890" };
        foreach (string text in bad)
        {
            string id; bool hasSeed; int seed; string error;
            Assert.IsFalse(ResetRequest.TryParse(text, out id, out hasSeed, out seed, out error), text);
            Assert.IsNotEmpty(error, text);
        }
    }

    [Test]
    public void The_exported_scenarios_json_parses_and_every_worker_has_a_valid_path()
    {
        string path = Path.Combine(Application.streamingAssetsPath, "scenarios.json");
        Assert.IsTrue(File.Exists(path), path);
        ScenarioFile file = ScenarioFile.FromJson(File.ReadAllText(path));

        Assert.IsNotNull(file.Find("S-04"));
        Assert.IsNotNull(file.Find("s-06"), "the lookup ignores case");
        Assert.IsNull(file.Find("S-99"));
        Assert.AreEqual(0, file.Find("S-00c").workers.Count);
        Assert.AreEqual(2, file.Find("S-08").workers.Count);
        foreach (ScenarioEntry entry in file.scenarios)
        {
            foreach (NpcWorkerSpec w in entry.workers) Assert.DoesNotThrow(() => new NpcPath(w), entry.id + " " + w.name);
        }

        // S-04 is the old hardcoded line: ROS (0, 1.5) to (0, -1.5) at 0.5 m/s, a 6 s leg.
        var s04 = new NpcPath(file.Find("S-04").workers[0]);
        Assert.AreEqual(1.5f, s04.PositionAt(0f).y, 1e-3);
        Assert.AreEqual(-1.5f, s04.PositionAt(6f).y, 1e-3);
    }

    [Test]
    public void A_file_with_another_schema_is_refused()
    {
        Assert.Throws<System.FormatException>(() => ScenarioFile.FromJson("{\"schema\":2,\"scenarios\":[]}"));
        Assert.Throws<System.FormatException>(() => ScenarioFile.FromJson("{\"schema\":1}"));
    }

    [Test]
    public void A_spec_worker_stands_at_its_first_waypoint_in_unity_coordinates_before_it_moves()
    {
        // ROS (x, y) -> Unity (X = -y * scale, Z = x * scale)
        var go = new GameObject("npc_test");
        try
        {
            var mover = go.AddComponent<NpcMover>();
            mover.InitFromSpec(new NpcPath(Worker("once", new[] { 1f, 2f }, new[] { 3f, 2f })), 4f, 1f);
            Assert.AreEqual(-8f, go.transform.position.x, 1e-3);
            Assert.AreEqual(1f, go.transform.position.y, 1e-3);
            Assert.AreEqual(4f, go.transform.position.z, 1e-3);
            Assert.AreEqual(1f, mover.RosPosition.Value.x, 1e-3);
            Assert.AreEqual(2f, mover.RosPosition.Value.y, 1e-3);
        }
        finally
        {
            Object.DestroyImmediate(go);
        }
    }

    [Test]
    public void A_legacy_worker_has_no_spec_position()
    {
        var go = new GameObject("npc_legacy");
        try
        {
            Assert.IsFalse(go.AddComponent<NpcMover>().RosPosition.HasValue);
        }
        finally
        {
            Object.DestroyImmediate(go);
        }
    }
}
