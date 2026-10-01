using NUnit.Framework;

public class PublishTimerTests
{
    [Test]
    public void Fires_on_first_tick()
    {
        var timer = new PublishTimer(10.0);
        Assert.IsTrue(timer.Tick(0.0));
    }

    [Test]
    public void Fires_once_per_period()
    {
        var timer = new PublishTimer(10.0);
        timer.Tick(0.0);
        Assert.IsFalse(timer.Tick(0.05));
        Assert.IsTrue(timer.Tick(0.1));
        Assert.IsFalse(timer.Tick(0.15));
        Assert.IsTrue(timer.Tick(0.2));
    }

    [Test]
    public void Resyncs_instead_of_bursting_after_a_long_frame()
    {
        var timer = new PublishTimer(10.0);
        timer.Tick(0.0);
        Assert.IsTrue(timer.Tick(0.35));
        Assert.IsFalse(timer.Tick(0.36));
        Assert.IsFalse(timer.Tick(0.44));
        Assert.IsTrue(timer.Tick(0.45));
    }

    [Test]
    public void Rejects_non_positive_rate()
    {
        Assert.Throws<System.ArgumentOutOfRangeException>(() => new PublishTimer(0.0));
    }
}
