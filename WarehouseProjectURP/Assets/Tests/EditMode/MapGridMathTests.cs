using NUnit.Framework;
using UnityEngine;

/// <summary>
/// The geometry of the static /map: Unity world to the ROS "map" frame in robot metres (ADR-010, ADR-012).
/// </summary>
public class MapGridMathTests
{
    private const float Scale = 4f;
    private const double Tol = 1e-9;

    // Unity x in [-40, 40], z in [-20, 60]  =>  ROS x in [-5, 15], ROS y in [-10, 10] (robot metres).
    private static MapGridSpec Warehouse(double cell, double padding)
    {
        return MapGridMath.FromUnityBounds(-40f, -20f, 40f, 60f, Scale, cell, padding);
    }

    [Test]
    public void FromUnityBounds_turns_unity_axes_into_ros_axes_in_robot_metres()
    {
        MapGridSpec s = Warehouse(0.5, 0.0);
        Assert.AreEqual(-5.0, s.OriginX, Tol);   // ROS x = Unity z / 4
        Assert.AreEqual(-10.0, s.OriginY, Tol);  // ROS y = -Unity x / 4
        Assert.AreEqual(40, s.Width);            // 20 m / 0.5 m
        Assert.AreEqual(40, s.Height);
        Assert.AreEqual(1600, s.Count);
    }

    [Test]
    public void FromUnityBounds_adds_padding_on_every_side_and_snaps_the_origin_to_the_cell_size()
    {
        MapGridSpec s = Warehouse(0.5, 1.0);
        Assert.AreEqual(-6.0, s.OriginX, Tol);
        Assert.AreEqual(-11.0, s.OriginY, Tol);
        Assert.AreEqual(44, s.Width);
        Assert.AreEqual(44, s.Height);

        MapGridSpec odd = MapGridMath.FromUnityBounds(-3f, -3f, 3f, 3f, Scale, 0.5, 0.3);
        Assert.AreEqual(0.0, (odd.OriginX / 0.5) % 1.0, Tol, "the origin is a multiple of the cell size");
    }

    [Test]
    public void FromUnityBounds_covers_the_bounds_at_the_real_cell_size()
    {
        MapGridSpec s = Warehouse(0.05, 1.0);
        Assert.LessOrEqual(s.OriginX, -6.0 + Tol);
        Assert.LessOrEqual(s.OriginY, -11.0 + Tol);
        Assert.GreaterOrEqual(s.OriginX + s.Width * s.Cell, 16.0 - Tol);
        Assert.GreaterOrEqual(s.OriginY + s.Height * s.Cell, 11.0 - Tol);
    }

    [Test]
    public void FromUnityBounds_rejects_bad_input()
    {
        Assert.Throws<System.ArgumentOutOfRangeException>(() => MapGridMath.FromUnityBounds(0f, 0f, 1f, 1f, 0f, 0.05, 0.0));
        Assert.Throws<System.ArgumentOutOfRangeException>(() => MapGridMath.FromUnityBounds(0f, 0f, 1f, 1f, Scale, 0.0, 0.0));
        Assert.Throws<System.ArgumentException>(() => MapGridMath.FromUnityBounds(1f, 0f, 0f, 1f, Scale, 0.05, 0.0));
    }

    [Test]
    public void CellCentreUnity_is_inside_the_unity_rectangle_at_the_ros_origin_corner()
    {
        MapGridSpec s = Warehouse(0.5, 0.0);
        Vector3 c = MapGridMath.CellCentreUnity(s, 0, 0, 1.5f);
        // Cell (0, 0) is the ROS minimum corner = Unity maximum x, minimum z. A cell is 2 Unity units wide.
        Assert.AreEqual(39f, c.x, 1e-4f);
        Assert.AreEqual(1.5f, c.y, 1e-6f);
        Assert.AreEqual(-19f, c.z, 1e-4f);
    }

    [Test]
    public void CellCentreUnity_and_ToMapPose_agree()
    {
        MapGridSpec s = Warehouse(0.5, 1.0);
        Vector3 unity = MapGridMath.CellCentreUnity(s, 7, 13, 0f);
        Pose2D ros = RosConversions.ToMapPose(unity, 0f, Scale);
        Assert.AreEqual(s.OriginX + 7.5 * s.Cell, ros.X, 1e-4);
        Assert.AreEqual(s.OriginY + 13.5 * s.Cell, ros.Y, 1e-4);
        Assert.AreEqual(13 * s.Width + 7, MapGridMath.CellIndex(s, ros.X, ros.Y));
    }

    [Test]
    public void CellIndex_is_minus_one_outside_the_grid()
    {
        MapGridSpec s = Warehouse(0.5, 0.0);
        Assert.AreEqual(0, MapGridMath.CellIndex(s, -4.9, -9.9));
        Assert.AreEqual(-1, MapGridMath.CellIndex(s, -5.1, 0.0));
        Assert.AreEqual(-1, MapGridMath.CellIndex(s, 0.0, 10.1));
        Assert.AreEqual(-1, MapGridMath.CellIndex(s, 15.0, 0.0));
    }

    [Test]
    public void Build_marks_exactly_the_blocked_cells()
    {
        MapGridSpec s = Warehouse(0.5, 0.0);
        sbyte[] data = MapGridMath.Build(s, (i, j) => i == j);
        Assert.AreEqual(s.Count, data.Length);
        for (int j = 0; j < s.Height; j++)
        {
            for (int i = 0; i < s.Width; i++)
            {
                Assert.AreEqual(i == j ? MapGridMath.Occupied : MapGridMath.Free, data[j * s.Width + i], "cell " + i + "," + j);
            }
        }
    }

    [Test]
    public void FillRow_touches_only_its_own_row()
    {
        MapGridSpec s = Warehouse(0.5, 0.0);
        sbyte[] data = new sbyte[s.Count];
        MapGridMath.FillRow(s, data, 3, (i, j) => true);
        for (int k = 0; k < data.Length; k++)
        {
            Assert.AreEqual(k / s.Width == 3 ? MapGridMath.Occupied : MapGridMath.Free, data[k]);
        }
    }

    // The warehouse above is square, so a width/height mix-up would pass every test. This one is not:
    // Unity x in [-40, 40], z in [-20, 100]  =>  ROS x in [-5, 25] (30 m), ROS y in [-10, 10] (20 m).
    private static MapGridSpec LongWarehouse()
    {
        return MapGridMath.FromUnityBounds(-40f, -20f, 40f, 100f, Scale, 0.5, 0.0);
    }

    [Test]
    public void A_non_square_warehouse_is_wider_in_ros_x_than_in_ros_y()
    {
        MapGridSpec s = LongWarehouse();
        Assert.AreEqual(60, s.Width);   // 30 m / 0.5 m along ROS x (Unity z)
        Assert.AreEqual(40, s.Height);  // 20 m / 0.5 m along ROS y (Unity x)
        Assert.AreEqual(2400, s.Count);
    }

    [Test]
    public void CellIndex_counts_along_the_width_in_a_non_square_warehouse()
    {
        MapGridSpec s = LongWarehouse();
        Assert.AreEqual(59, MapGridMath.CellIndex(s, 24.9, -9.9));            // last column, first row
        Assert.AreEqual(39 * 60, MapGridMath.CellIndex(s, -4.9, 9.9));        // first column, last row
        Assert.AreEqual(-1, MapGridMath.CellIndex(s, 25.1, 0.0));
        Assert.AreEqual(-1, MapGridMath.CellIndex(s, 0.0, 10.1));
    }

    [Test]
    public void Build_and_FillRow_use_the_width_as_the_row_length_in_a_non_square_warehouse()
    {
        MapGridSpec s = LongWarehouse();
        sbyte[] data = MapGridMath.Build(s, (i, j) => i == 59 && j == 39);   // the far corner only
        Assert.AreEqual(MapGridMath.Occupied, data[s.Count - 1]);
        int occupied = 0;
        for (int k = 0; k < data.Length; k++) if (data[k] == MapGridMath.Occupied) occupied++;
        Assert.AreEqual(1, occupied);

        sbyte[] row = new sbyte[s.Count];
        MapGridMath.FillRow(s, row, 2, (i, j) => true);
        for (int k = 0; k < row.Length; k++)
        {
            Assert.AreEqual(k / 60 == 2 ? MapGridMath.Occupied : MapGridMath.Free, row[k], "index " + k);
        }
    }

    [Test]
    public void CellCentreUnity_and_ToMapPose_agree_in_a_non_square_warehouse()
    {
        MapGridSpec s = LongWarehouse();
        Vector3 unity = MapGridMath.CellCentreUnity(s, 47, 11, 0f);
        Pose2D ros = RosConversions.ToMapPose(unity, 0f, Scale);
        Assert.AreEqual(s.OriginX + 47.5 * s.Cell, ros.X, 1e-4);
        Assert.AreEqual(s.OriginY + 11.5 * s.Cell, ros.Y, 1e-4);
        Assert.AreEqual(11 * 60 + 47, MapGridMath.CellIndex(s, ros.X, ros.Y));
    }
}
