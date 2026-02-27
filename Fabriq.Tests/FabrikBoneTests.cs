using System.Numerics;
using Fabriq;

namespace Fabriq.Tests;

public class FabrikBoneTests
{
    // -------------------------------------------------------------------------
    // Construction
    // -------------------------------------------------------------------------

    [Fact]
    public void Constructor_WithLength_HasIdentityOrientationAndCorrectLength()
    {
        var bone = new FabrikBone(2.5f);
        Assert.Equal(Quaternion.Identity, bone.Orientation);
        Assert.Equal(2.5f, bone.Length);
    }

    [Fact]
    public void Constructor_WithOrientationAndLength_StoresNormalisedOrientation()
    {
        // Supply an unnormalised quaternion — should be normalised on set.
        var raw = new Quaternion(1f, 1f, 1f, 1f);
        var bone = new FabrikBone(raw, 1.0f);
        Assert.True(MathF.Abs(bone.Orientation.Length() - 1.0f) < 1e-5f,
            "Orientation should be normalised.");
    }

    [Fact]
    public void Constructor_WithZeroLength_Throws()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new FabrikBone(0f));
    }

    [Fact]
    public void Constructor_WithNegativeLength_Throws()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new FabrikBone(-1f));
    }

    [Fact]
    public void Constructor_WithZeroQuaternion_Throws()
    {
        Assert.Throws<ArgumentException>(() =>
            new FabrikBone(new Quaternion(0f, 0f, 0f, 0f), 1.0f));
    }

    [Fact]
    public void CopyConstructor_ProducesIndependentCopy()
    {
        var original = new FabrikBone(Quaternion.Identity, 3.0f, "original");
        var copy = new FabrikBone(original);

        Assert.Equal(original.Orientation, copy.Orientation);
        Assert.Equal(original.Length, copy.Length);
        Assert.Equal(original.Name, copy.Name);

        // Mutating the copy must not affect the original.
        copy.Length = 5.0f;
        copy.Name = "copy";
        Assert.Equal(3.0f, original.Length);
        Assert.Equal("original", original.Name);
    }

    [Fact]
    public void CopyConstructor_WithNull_Throws()
    {
        Assert.Throws<ArgumentNullException>(() => new FabrikBone(null!));
    }

    // -------------------------------------------------------------------------
    // Factory: FromDirection
    // -------------------------------------------------------------------------

    [Fact]
    public void FromDirection_UnitY_ProducesIdentityOrientation()
    {
        // A bone asked to point in +Y should require no rotation from rest.
        var bone = FabrikBone.FromDirection(Vector3.UnitY, 1.0f);
        Assert.True(MathF.Abs(bone.Orientation.Length() - 1.0f) < 1e-5f);

        // Rotating +Y by identity should still give +Y.
        var tip = Vector3.Transform(Vector3.UnitY, bone.Orientation);
        Assert.True(Vector3.Distance(tip, Vector3.UnitY) < 1e-5f,
            "Bone tip should point in +Y for identity orientation.");
    }

    [Fact]
    public void FromDirection_UnitX_BoneTipPointsAlongX()
    {
        var bone = FabrikBone.FromDirection(Vector3.UnitX, 1.0f);
        var tip = Vector3.Transform(Vector3.UnitY, bone.Orientation);
        Assert.True(Vector3.Distance(tip, Vector3.UnitX) < 1e-5f,
            "Bone tip should point in +X direction.");
    }

    [Fact]
    public void FromDirection_UnitZ_BoneTipPointsAlongZ()
    {
        var bone = FabrikBone.FromDirection(Vector3.UnitZ, 1.0f);
        var tip = Vector3.Transform(Vector3.UnitY, bone.Orientation);
        Assert.True(Vector3.Distance(tip, Vector3.UnitZ) < 1e-5f,
            "Bone tip should point in +Z direction.");
    }

    [Fact]
    public void FromDirection_NegativeY_BoneTipPointsDown()
    {
        // Antiparallel case — the degenerate 180° rotation path.
        var bone = FabrikBone.FromDirection(-Vector3.UnitY, 1.0f);
        var tip = Vector3.Transform(Vector3.UnitY, bone.Orientation);
        Assert.True(Vector3.Distance(tip, -Vector3.UnitY) < 1e-5f,
            "Bone tip should point in -Y direction.");
    }

    [Fact]
    public void FromDirection_UnnormalisedInput_StillProducesCorrectOrientation()
    {
        // Direction of length 5 pointing along +X — should behave the same
        // as a unit vector.
        var bone = FabrikBone.FromDirection(new Vector3(5f, 0f, 0f), 1.0f);
        var tip = Vector3.Transform(Vector3.UnitY, bone.Orientation);
        Assert.True(Vector3.Distance(tip, Vector3.UnitX) < 1e-5f);
    }

    [Fact]
    public void FromDirection_ZeroVector_Throws()
    {
        Assert.Throws<ArgumentException>(() =>
            FabrikBone.FromDirection(Vector3.Zero, 1.0f));
    }

    // -------------------------------------------------------------------------
    // Factory: For2D
    // -------------------------------------------------------------------------

    [Fact]
    public void For2D_ZeroAngle_PointsAlongPositiveX()
    {
        var bone = FabrikBone.For2D(0f, 1.0f);
        var tip = Vector3.Transform(Vector3.UnitY, bone.Orientation);
        Assert.True(Vector3.Distance(tip, Vector3.UnitX) < 1e-5f,
            "0 radians should point along +X.");
    }

    [Fact]
    public void For2D_HalfPi_PointsAlongPositiveY()
    {
        var bone = FabrikBone.For2D(MathF.PI / 2f, 1.0f);
        var tip = Vector3.Transform(Vector3.UnitY, bone.Orientation);
        Assert.True(Vector3.Distance(tip, Vector3.UnitY) < 1e-5f,
            "PI/2 radians should point along +Y.");
    }

    [Fact]
    public void For2D_BoneLiesInXYPlane()
    {
        // For any angle, the bone's tip should have no Z component.
        foreach (var angle in new[] { 0f, 0.5f, 1.0f, MathF.PI, 3f, MathF.PI * 2f - 0.1f })
        {
            var bone = FabrikBone.For2D(angle, 1.0f);
            var tip = Vector3.Transform(Vector3.UnitY, bone.Orientation);
            Assert.True(MathF.Abs(tip.Z) < 1e-5f,
                $"Bone at angle {angle} rad should lie in XY plane (Z≈0), got Z={tip.Z}.");
        }
    }

    // -------------------------------------------------------------------------
    // Property validation
    // -------------------------------------------------------------------------

    [Fact]
    public void SetLength_ToPositiveValue_Succeeds()
    {
        var bone = new FabrikBone(1.0f);
        bone.Length = 99.9f;
        Assert.Equal(99.9f, bone.Length);
    }

    [Fact]
    public void SetLength_ToZero_Throws()
    {
        var bone = new FabrikBone(1.0f);
        Assert.Throws<ArgumentOutOfRangeException>(() => bone.Length = 0f);
    }

    [Fact]
    public void SetOrientation_IsNormalised()
    {
        var bone = new FabrikBone(1.0f);
        bone.Orientation = new Quaternion(2f, 0f, 0f, 2f);
        Assert.True(MathF.Abs(bone.Orientation.Length() - 1.0f) < 1e-5f);
    }

    // -------------------------------------------------------------------------
    // ToString
    // -------------------------------------------------------------------------

    [Fact]
    public void ToString_WithName_ContainsName()
    {
        var bone = new FabrikBone(Quaternion.Identity, 1.0f, "spine");
        Assert.Contains("spine", bone.ToString());
    }

    // ToString_WithConstraint_ContainsConstraintType — added when RotorConstraint is implemented.

    [Fact]
    public void ToString_NoConstraint_ContainsNone()
    {
        var bone = new FabrikBone(1.0f);
        Assert.Contains("None", bone.ToString());
    }
}
