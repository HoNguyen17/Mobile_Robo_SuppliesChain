/// <summary>A planar pose in ROS conventions: metres, and yaw in radians (counter-clockwise, in (-pi, pi]).</summary>
public readonly struct Pose2D
{
    public readonly double X;
    public readonly double Y;
    public readonly double Yaw;

    public Pose2D(double x, double y, double yaw)
    {
        X = x;
        Y = y;
        Yaw = yaw;
    }

    public override string ToString()
    {
        return "(" + X.ToString("F3") + ", " + Y.ToString("F3") + ", yaw " + Yaw.ToString("F3") + ")";
    }
}
