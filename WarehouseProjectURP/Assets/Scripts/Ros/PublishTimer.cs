using System;

/// <summary>
/// Decides when a publisher should send, at a fixed rate in simulation time. After a long frame it sends once
/// and resynchronises, instead of sending a burst of catch-up messages.
/// </summary>
public sealed class PublishTimer
{
    private readonly double period;
    private double next = double.NegativeInfinity;

    public PublishTimer(double rateHz)
    {
        if (rateHz <= 0.0) throw new ArgumentOutOfRangeException(nameof(rateHz), rateHz, "Rate must be positive.");
        period = 1.0 / rateHz;
    }

    /// <summary>True when a message is due at simulation time <paramref name="now"/> (seconds).</summary>
    public bool Tick(double now)
    {
        // Small tolerance so 0.1 + 0.1 + 0.1 still counts as 0.3 despite floating-point error.
        if (now < next - 1e-9) return false;
        next = now - next > period ? now + period : next + period;
        return true;
    }
}
