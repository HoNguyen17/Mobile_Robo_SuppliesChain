using System.Collections.Generic;
using System.Globalization;

/// <summary>
/// The pure part of the collision report (docs/architecture.md section 5): which contacts to report again,
/// how an object is named, and the JSON text for /sim/collision. No Unity objects, so it is unit-tested.
/// </summary>
public sealed class CollisionBook
{
    private readonly double debounceSeconds;
    private readonly Dictionary<int, double> lastReported = new Dictionary<int, double>();

    /// <param name="debounceSeconds">A second contact with the same object within this time is not reported again.</param>
    public CollisionBook(double debounceSeconds = 1.0)
    {
        this.debounceSeconds = debounceSeconds;
    }

    /// <summary>True when a contact with <paramref name="otherId"/> at simulation time <paramref name="now"/> is a new collision.</summary>
    public bool ShouldReport(int otherId, double now)
    {
        double last;
        if (lastReported.TryGetValue(otherId, out last) && now - last < debounceSeconds) return false;
        lastReported[otherId] = now;
        return true;
    }

    /// <summary>
    /// Index of the first real tag in <paramref name="tagsFromColliderUp"/> (the tag of the collider's object, then
    /// of each parent up to its owner), or -1 when nothing is tagged. The hit collider is often a leaf: a rack leg
    /// is Untagged, only the rack root carries "Shelf".
    /// </summary>
    public static int FirstTaggedIndex(IList<string> tagsFromColliderUp)
    {
        if (tagsFromColliderUp == null) return -1;
        for (int i = 0; i < tagsFromColliderUp.Count; i++)
        {
            string tag = tagsFromColliderUp[i];
            if (!string.IsNullOrEmpty(tag) && tag != UntaggedTag) return i;
        }
        return -1;
    }

    private const string UntaggedTag = "Untagged";

    /// <summary>The allowed values of other_tag: Wall, Shelf, Obstacle, NPC.</summary>
    public static string OtherTag(string unityTag)
    {
        switch (unityTag)
        {
            case "WallPanel": return "Wall";
            case "Shelf": return "Shelf";
            case "NPC": return "NPC";
            default: return "Obstacle";
        }
    }

    /// <summary>
    /// {"other_tag":..., "sim_time_s":..., "x":..., "y":...}. Always a decimal point, whatever the computer's
    /// language is, because the reader is Python's json.
    /// </summary>
    public static string ToJson(string otherTag, double simTimeS, double x, double y)
    {
        return string.Format(CultureInfo.InvariantCulture,
            "{{\"other_tag\":\"{0}\",\"sim_time_s\":{1},\"x\":{2},\"y\":{3}}}",
            otherTag, Number(simTimeS), Number(x), Number(y));
    }

    private static string Number(double value)
    {
        if (double.IsNaN(value) || double.IsInfinity(value)) value = 0.0;
        return value.ToString("0.###", CultureInfo.InvariantCulture);
    }
}
