using System;
using UnityEngine;

/// <summary>
/// Holding brake for the wheel motors. Real Dynamixel servos hold their position when told to stop;
/// a pure velocity drive only resists speed, so the robot slowly creeps on any tiny slope.
/// </summary>
public static class WheelHold
{
    private const float Epsilon = 1e-4f;

    /// <summary>
    /// Hold only when nothing is commanded AND the speed ramp has reached zero, so the brake never fights
    /// the controlled deceleration.
    /// </summary>
    public static bool ShouldHold(float commandLinear, float commandAngular, float rampLinear, float rampAngular)
    {
        return Mathf.Abs(commandLinear) < Epsilon && Mathf.Abs(commandAngular) < Epsilon &&
               Mathf.Abs(rampLinear) < Epsilon && Mathf.Abs(rampAngular) < Epsilon;
    }

    /// <summary>
    /// Position stiffness (Unity ArticulationDrive units: torque per degree) that reaches
    /// <paramref name="torqueLimit"/> at <paramref name="fullTorqueAtDeg"/> degrees of error.
    /// </summary>
    public static float Stiffness(float torqueLimit, float fullTorqueAtDeg)
    {
        if (fullTorqueAtDeg <= 0f)
        {
            throw new ArgumentOutOfRangeException(nameof(fullTorqueAtDeg), fullTorqueAtDeg, "Angle must be positive.");
        }
        return torqueLimit / fullTorqueAtDeg;
    }
}
