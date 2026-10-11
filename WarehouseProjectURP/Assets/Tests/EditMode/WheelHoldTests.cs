using NUnit.Framework;

/// <summary>Holding brake: the wheel motors lock their position once the robot is commanded to stop.</summary>
public class WheelHoldTests
{
    [Test]
    public void Holds_when_command_and_ramp_are_zero()
    {
        Assert.IsTrue(WheelHold.ShouldHold(0f, 0f, 0f, 0f));
    }

    [Test]
    public void Does_not_hold_while_still_ramping_down()
    {
        Assert.IsFalse(WheelHold.ShouldHold(0f, 0f, 0.05f, 0f));
        Assert.IsFalse(WheelHold.ShouldHold(0f, 0f, 0f, -0.2f));
    }

    [Test]
    public void Releases_as_soon_as_a_command_arrives()
    {
        Assert.IsFalse(WheelHold.ShouldHold(0.1f, 0f, 0f, 0f));
        Assert.IsFalse(WheelHold.ShouldHold(0f, 0.5f, 0f, 0f));
    }

    [Test]
    public void Stiffness_reaches_the_torque_limit_at_the_given_angle()
    {
        Assert.AreEqual(256f, WheelHold.Stiffness(512f, 2f), 1e-4f);
    }

    [Test]
    public void Stiffness_guards_against_a_zero_angle()
    {
        Assert.Throws<System.ArgumentOutOfRangeException>(() => WheelHold.Stiffness(512f, 0f));
    }
}
