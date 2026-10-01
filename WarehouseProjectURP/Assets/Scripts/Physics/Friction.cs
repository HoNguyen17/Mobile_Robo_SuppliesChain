/// <summary>Static and dynamic friction coefficients of one surface.</summary>
public readonly struct Friction
{
    public readonly float Static;
    public readonly float Dynamic;

    public Friction(float staticFriction, float dynamicFriction)
    {
        Static = staticFriction;
        Dynamic = dynamicFriction;
    }
}
