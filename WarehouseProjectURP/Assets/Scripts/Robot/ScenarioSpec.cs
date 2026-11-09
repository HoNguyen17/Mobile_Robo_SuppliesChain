using System;
using System.Collections.Generic;
using UnityEngine;

// The data of a spec-driven scenario (ADR-019), exactly the JSON that
// ros1/warehouse_eval/src/warehouse_eval/scenario_spec.py writes to Assets/StreamingAssets/scenarios.json.
// Field names are the JSON names, because JsonUtility maps them one to one. Lengths are ROS metres.

[Serializable]
public class NpcWaypoint
{
    public float x;
    public float y;
    /// <summary>Seconds to wait here after arriving; -1 = for ever.</summary>
    public float dwell_s;
}

[Serializable]
public class NpcWorkerSpec
{
    public string name;
    public float speed_mps;
    public float radius_m;
    /// <summary>"once" (holds the last waypoint) or "pingpong".</summary>
    public string mode;
    /// <summary>"robot_departs" or "reset".</summary>
    public string start_trigger;
    public float start_delay_s;
    public float start_jitter_s;
    public List<NpcWaypoint> waypoints;
}

[Serializable]
public class ScenarioEntry
{
    public string id;
    public string name;
    public List<NpcWorkerSpec> workers;
}

[Serializable]
public class ScenarioFile
{
    public int schema;
    public List<ScenarioEntry> scenarios;

    public const int SupportedSchema = 1;

    /// <summary>Parses scenarios.json. Throws FormatException when the text is not a supported scenario file.</summary>
    public static ScenarioFile FromJson(string json)
    {
        ScenarioFile file = null;
        try { file = JsonUtility.FromJson<ScenarioFile>(json); }
        catch (ArgumentException ex) { throw new FormatException("scenarios.json is not valid JSON: " + ex.Message); }
        if (file == null) throw new FormatException("scenarios.json has no scenarios");
        if (file.schema != SupportedSchema) throw new FormatException("scenarios.json schema " + file.schema + " is not supported");
        // JsonUtility leaves a missing list as an empty list, not null, so an empty list counts as "no scenarios".
        if (file.scenarios == null || file.scenarios.Count == 0) throw new FormatException("scenarios.json has no scenarios");
        return file;
    }

    /// <summary>The scenario with this id (case does not matter), or null.</summary>
    public ScenarioEntry Find(string id)
    {
        if (scenarios == null || id == null) return null;
        foreach (ScenarioEntry entry in scenarios)
        {
            if (string.Equals(entry.id, id, StringComparison.OrdinalIgnoreCase)) return entry;
        }
        return null;
    }
}

/// <summary>The /sim/reset_scenario text: "ID" (the old protocol) or "ID?seed=N". Same rules as scenario_spec.parse_reset_id.</summary>
public static class ResetRequest
{
    /// <summary>
    /// False only for a malformed message (<paramref name="error"/> says why). <paramref name="hasSeed"/> is false for a
    /// plain ID, which must take the old code path.
    /// </summary>
    public static bool TryParse(string text, out string id, out bool hasSeed, out int seed, out string error)
    {
        id = (text ?? string.Empty).Trim();
        hasSeed = false;
        seed = 0;
        error = null;
        int q = id.IndexOf('?');
        if (q < 0) return true;

        string query = id.Substring(q + 1);
        id = id.Substring(0, q).Trim();
        const string prefix = "seed=";
        if (id.Length == 0 || !query.StartsWith(prefix, StringComparison.Ordinal))
        {
            error = "reset message must be ID or ID?seed=N";
            return false;
        }
        string digits = query.Substring(prefix.Length);
        if (digits.Length == 0 || digits.Length > 9 || !AllDigits(digits))
        {
            error = "seed must be a non-negative integer";
            return false;
        }
        hasSeed = true;
        seed = int.Parse(digits, System.Globalization.CultureInfo.InvariantCulture);
        return true;
    }

    private static bool AllDigits(string s)
    {
        foreach (char c in s)
        {
            if (c < '0' || c > '9') return false;
        }
        return true;
    }
}
