using System;
using UnityEngine;

/// <summary>
/// Size and placement of the static /map in the ROS "map" frame. Lengths are robot metres (ADR-010).
/// Plain data, so the geometry can be unit-tested without a scene.
/// </summary>
public readonly struct MapGridSpec
{
    public readonly int Width;
    public readonly int Height;
    /// <summary>Cell size in robot metres.</summary>
    public readonly double Cell;
    /// <summary>ROS x of the lower-left corner of cell (0, 0).</summary>
    public readonly double OriginX;
    /// <summary>ROS y of the lower-left corner of cell (0, 0).</summary>
    public readonly double OriginY;
    /// <summary>Unity units per robot metre.</summary>
    public readonly float Scale;

    public MapGridSpec(int width, int height, double cell, double originX, double originY, float scale)
    {
        Width = width;
        Height = height;
        Cell = cell;
        OriginX = originX;
        OriginY = originY;
        Scale = scale;
    }

    public int Count
    {
        get { return Width * Height; }
    }
}

/// <summary>
/// The geometry of the static map (ADR-012). The map frame is Unity's world turned into ROS axes
/// (ROS x = Unity Z, ROS y = -Unity X) and divided by the robot scale, so its origin is the Unity world origin.
/// Cell (i, j) covers ROS x from OriginX + i * Cell and y from OriginY + j * Cell; data[j * Width + i].
/// </summary>
public static class MapGridMath
{
    public const sbyte Free = 0;
    public const sbyte Occupied = 100;

    /// <summary>
    /// A grid that covers the given Unity world rectangle plus <paramref name="paddingM"/> robot metres on every side.
    /// The origin is a multiple of the cell size, so the same warehouse always gives the same grid.
    /// </summary>
    public static MapGridSpec FromUnityBounds(float minX, float minZ, float maxX, float maxZ,
                                              float robotScale, double cellM, double paddingM)
    {
        if (!(robotScale > 0f)) throw new ArgumentOutOfRangeException(nameof(robotScale), robotScale, "Scale must be positive.");
        if (!(cellM > 0.0)) throw new ArgumentOutOfRangeException(nameof(cellM), cellM, "Cell size must be positive.");
        if (maxX < minX || maxZ < minZ) throw new ArgumentException("The bounds are empty.");

        double rosMinX = minZ / robotScale;
        double rosMaxX = maxZ / robotScale;
        double rosMinY = -maxX / robotScale;
        double rosMaxY = -minX / robotScale;

        double originX = Math.Floor((rosMinX - paddingM) / cellM) * cellM;
        double originY = Math.Floor((rosMinY - paddingM) / cellM) * cellM;
        int width = (int)Math.Ceiling((rosMaxX + paddingM - originX) / cellM);
        int height = (int)Math.Ceiling((rosMaxY + paddingM - originY) / cellM);
        return new MapGridSpec(width, height, cellM, originX, originY, robotScale);
    }

    /// <summary>Unity world position of the centre of cell (i, j), at height <paramref name="y"/>.</summary>
    public static Vector3 CellCentreUnity(MapGridSpec spec, int i, int j, float y)
    {
        double rosX = spec.OriginX + (i + 0.5) * spec.Cell;
        double rosY = spec.OriginY + (j + 0.5) * spec.Cell;
        return new Vector3((float)(-rosY * spec.Scale), y, (float)(rosX * spec.Scale));
    }

    /// <summary>Index of the cell that contains the ROS point (x, y), or -1 outside the grid.</summary>
    public static int CellIndex(MapGridSpec spec, double x, double y)
    {
        int i = (int)Math.Floor((x - spec.OriginX) / spec.Cell);
        int j = (int)Math.Floor((y - spec.OriginY) / spec.Cell);
        if (i < 0 || j < 0 || i >= spec.Width || j >= spec.Height) return -1;
        return j * spec.Width + i;
    }

    /// <summary>Fills row <paramref name="j"/> of <paramref name="data"/>: Occupied where <paramref name="isBlocked"/> is true.</summary>
    public static void FillRow(MapGridSpec spec, sbyte[] data, int j, Func<int, int, bool> isBlocked)
    {
        for (int i = 0; i < spec.Width; i++)
        {
            data[j * spec.Width + i] = isBlocked(i, j) ? Occupied : Free;
        }
    }

    /// <summary>The whole grid in one go.</summary>
    public static sbyte[] Build(MapGridSpec spec, Func<int, int, bool> isBlocked)
    {
        sbyte[] data = new sbyte[spec.Count];
        for (int j = 0; j < spec.Height; j++) FillRow(spec, data, j, isBlocked);
        return data;
    }
}
