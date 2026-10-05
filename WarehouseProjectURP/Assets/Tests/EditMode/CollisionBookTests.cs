using System.Globalization;
using System.Threading;
using NUnit.Framework;

/// <summary>The pure part of the collision report (docs/architecture.md section 5).</summary>
public class CollisionBookTests
{
    [Test]
    public void The_first_contact_is_reported()
    {
        Assert.IsTrue(new CollisionBook().ShouldReport(42, 10.0));
    }

    [Test]
    public void A_repeat_contact_with_the_same_object_within_the_debounce_time_is_not_reported()
    {
        CollisionBook book = new CollisionBook(1.0);
        Assert.IsTrue(book.ShouldReport(42, 10.0));
        Assert.IsFalse(book.ShouldReport(42, 10.4));
        Assert.IsFalse(book.ShouldReport(42, 10.99));
    }

    [Test]
    public void The_same_object_is_reported_again_after_the_debounce_time()
    {
        CollisionBook book = new CollisionBook(1.0);
        Assert.IsTrue(book.ShouldReport(42, 10.0));
        Assert.IsTrue(book.ShouldReport(42, 11.0));
    }

    [Test]
    public void The_debounce_time_counts_from_the_last_report_only()
    {
        CollisionBook book = new CollisionBook(1.0);
        Assert.IsTrue(book.ShouldReport(42, 10.0));
        Assert.IsFalse(book.ShouldReport(42, 10.9)); // not reported: does not restart the wait
        Assert.IsTrue(book.ShouldReport(42, 11.0));
    }

    [Test]
    public void Different_objects_are_independent()
    {
        CollisionBook book = new CollisionBook(1.0);
        Assert.IsTrue(book.ShouldReport(1, 10.0));
        Assert.IsTrue(book.ShouldReport(2, 10.1));
        Assert.IsFalse(book.ShouldReport(1, 10.2));
    }

    [TestCase("WallPanel", "Wall")]
    [TestCase("Shelf", "Shelf")]
    [TestCase("NPC", "NPC")]
    [TestCase("Untagged", "Obstacle")]
    [TestCase("Column", "Obstacle")]
    [TestCase("", "Obstacle")]
    public void OtherTag_uses_only_the_allowed_values(string unityTag, string expected)
    {
        Assert.AreEqual(expected, CollisionBook.OtherTag(unityTag));
    }

    // A rack is 7 untagged colliders (legs, boards) under one root that carries the Shelf tag. A wall panel
    // has the tag on the collider's own object. Tags are listed from the collider up to its owner.
    [Test]
    public void FirstTaggedIndex_skips_untagged_parts_up_to_the_owner_tag()
    {
        Assert.AreEqual(1, CollisionBook.FirstTaggedIndex(new[] { "Untagged", "Shelf" }));
        Assert.AreEqual(2, CollisionBook.FirstTaggedIndex(new[] { "Untagged", "Untagged", "Shelf" }));
    }

    [Test]
    public void FirstTaggedIndex_is_zero_when_the_collider_itself_is_tagged()
    {
        Assert.AreEqual(0, CollisionBook.FirstTaggedIndex(new[] { "WallPanel", "Untagged" }));
    }

    [Test]
    public void FirstTaggedIndex_takes_the_nearest_tag_not_the_outermost()
    {
        Assert.AreEqual(0, CollisionBook.FirstTaggedIndex(new[] { "Column", "Shelf" }));
    }

    [Test]
    public void FirstTaggedIndex_is_minus_one_when_nothing_is_tagged()
    {
        Assert.AreEqual(-1, CollisionBook.FirstTaggedIndex(new[] { "Untagged", "Untagged" }));
        Assert.AreEqual(-1, CollisionBook.FirstTaggedIndex(new[] { "", null }));
        Assert.AreEqual(-1, CollisionBook.FirstTaggedIndex(new string[0]));
        Assert.AreEqual(-1, CollisionBook.FirstTaggedIndex(null));
    }

    [Test]
    public void ToJson_has_the_four_fields_of_the_contract()
    {
        Assert.AreEqual("{\"other_tag\":\"Shelf\",\"sim_time_s\":12.5,\"x\":1.25,\"y\":-3.5}",
                        CollisionBook.ToJson("Shelf", 12.5, 1.25, -3.5));
    }

    [Test]
    public void ToJson_rounds_to_millimetres_and_keeps_whole_numbers_short()
    {
        Assert.AreEqual("{\"other_tag\":\"Wall\",\"sim_time_s\":3,\"x\":0.123,\"y\":0}",
                        CollisionBook.ToJson("Wall", 3.0, 0.12345, 0.0));
    }

    [Test]
    public void ToJson_uses_a_decimal_point_whatever_the_computer_language_is()
    {
        CultureInfo before = Thread.CurrentThread.CurrentCulture;
        try
        {
            Thread.CurrentThread.CurrentCulture = new CultureInfo("de-DE"); // decimal comma
            Assert.AreEqual("{\"other_tag\":\"Wall\",\"sim_time_s\":1.5,\"x\":2.5,\"y\":-0.5}",
                            CollisionBook.ToJson("Wall", 1.5, 2.5, -0.5));
        }
        finally
        {
            Thread.CurrentThread.CurrentCulture = before;
        }
    }

    [Test]
    public void ToJson_never_writes_nan_or_infinity()
    {
        string json = CollisionBook.ToJson("Wall", double.NaN, double.PositiveInfinity, double.NegativeInfinity);
        Assert.AreEqual("{\"other_tag\":\"Wall\",\"sim_time_s\":0,\"x\":0,\"y\":0}", json);
    }
}
