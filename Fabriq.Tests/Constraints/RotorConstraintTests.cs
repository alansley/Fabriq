using System.Numerics;
using Fabriq;

namespace Fabriq.Tests.Constraints;

/// <summary>
/// Tests for all five rotor constraint implementations.
///
/// Each implementation is tested independently for correctness, then a
/// cross-implementation comparison verifies they all agree on the core
/// invariants — particularly that a clamped orientation always falls within
/// the specified cone angle.
/// </summary>
public class RotorConstraintTests
{
    // The five implementations under test.
    public static IEnumerable<object[]> AllRotorConstraints(float coneAngle) =>
    [
        [new SwingTwistRotorConstraint(coneAngle)],
        [new AxisAngleRotorConstraint(coneAngle)],
        [new DotProductRotorConstraint(coneAngle)],
        [new ExponentialMapRotorConstraint(coneAngle)],
        [new CGARotorConstraint(coneAngle)],
    ];

    private static readonly Quaternion ParentIdentity = Quaternion.Identity;

    // -------------------------------------------------------------------------
    // Construction and validation
    // -------------------------------------------------------------------------

    [Fact]
    public void AllConstraints_HaveRotorType()
    {
        var coneAngle = MathF.PI / 4f;
        Assert.Equal(ConstraintType.Rotor, new SwingTwistRotorConstraint(coneAngle).Type);
        Assert.Equal(ConstraintType.Rotor, new AxisAngleRotorConstraint(coneAngle).Type);
        Assert.Equal(ConstraintType.Rotor, new DotProductRotorConstraint(coneAngle).Type);
        Assert.Equal(ConstraintType.Rotor, new ExponentialMapRotorConstraint(coneAngle).Type);
        Assert.Equal(ConstraintType.Rotor, new CGARotorConstraint(coneAngle).Type);
    }

    [Theory]
    [InlineData(0f)]
    [InlineData(-0.1f)]
    [InlineData(MathF.PI + 0.001f)]
    public void AllConstraints_InvalidConeAngle_Throws(float badAngle)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new SwingTwistRotorConstraint(badAngle));
        Assert.Throws<ArgumentOutOfRangeException>(() => new AxisAngleRotorConstraint(badAngle));
        Assert.Throws<ArgumentOutOfRangeException>(() => new DotProductRotorConstraint(badAngle));
        Assert.Throws<ArgumentOutOfRangeException>(() => new ExponentialMapRotorConstraint(badAngle));
        Assert.Throws<ArgumentOutOfRangeException>(() => new CGARotorConstraint(badAngle));
    }

    // -------------------------------------------------------------------------
    // Core invariant: orientation within cone passes through unchanged
    // -------------------------------------------------------------------------

    [Theory]
    [MemberData(nameof(AllRotorConstraints), MathF.PI / 4f)]
    public void Clamp_OrientationWithinCone_ReturnsUnchanged(IFabrikConstraint constraint)
    {
        // Rotate 20° around X — well within a 45° cone.
        var orientation = Quaternion.CreateFromAxisAngle(Vector3.UnitX, 20f * MathF.PI / 180f);
        var clamped = constraint.Clamp(orientation, ParentIdentity);

        // The clamped result should be essentially the same quaternion.
        var diff = Quaternion.Dot(orientation, clamped);
        Assert.True(MathF.Abs(diff) > 0.9999f,
            $"{constraint.GetType().Name}: orientation within cone should not be modified.");
    }

    [Theory]
    [MemberData(nameof(AllRotorConstraints), MathF.PI / 4f)]
    public void Clamp_IdentityOrientation_ReturnsIdentity(IFabrikConstraint constraint)
    {
        var clamped = constraint.Clamp(Quaternion.Identity, ParentIdentity);
        var diff = Quaternion.Dot(Quaternion.Identity, clamped);
        Assert.True(MathF.Abs(diff) > 0.9999f,
            $"{constraint.GetType().Name}: identity orientation should remain identity.");
    }

    // -------------------------------------------------------------------------
    // Core invariant: clamped orientation falls within the cone
    // -------------------------------------------------------------------------

    [Theory]
    [MemberData(nameof(AllRotorConstraints), MathF.PI / 4f)]
    public void Clamp_OrientationOutsideCone_ResultIsWithinCone(IFabrikConstraint constraint)
    {
        var coneAngle = MathF.PI / 4f; // 45°

        // Rotate 80° around X — well outside a 45° cone.
        var orientation = Quaternion.CreateFromAxisAngle(Vector3.UnitX, 80f * MathF.PI / 180f);
        var clamped = constraint.Clamp(orientation, ParentIdentity);

        var clampedDirection = Vector3.Transform(Vector3.UnitY, clamped);
        var dot = Math.Clamp(Vector3.Dot(clampedDirection, Vector3.UnitY), -1f, 1f);
        var resultAngle = MathF.Acos(dot);

        Assert.True(resultAngle <= coneAngle + 1e-4f,
            $"{constraint.GetType().Name}: clamped angle {resultAngle:F4} rad exceeds " +
            $"cone angle {coneAngle:F4} rad.");
    }

    [Theory]
    [MemberData(nameof(AllRotorConstraints), MathF.PI / 6f)]
    public void Clamp_LargeConeViolation_ResultIsWithinCone(IFabrikConstraint constraint)
    {
        var coneAngle = MathF.PI / 6f; // 30°

        // Rotate 170° — nearly pointing backward.
        var orientation = Quaternion.CreateFromAxisAngle(Vector3.UnitX, 170f * MathF.PI / 180f);
        var clamped = constraint.Clamp(orientation, ParentIdentity);

        var clampedDirection = Vector3.Transform(Vector3.UnitY, clamped);
        var dot = Math.Clamp(Vector3.Dot(clampedDirection, Vector3.UnitY), -1f, 1f);
        var resultAngle = MathF.Acos(dot);

        Assert.True(resultAngle <= coneAngle + 1e-4f,
            $"{constraint.GetType().Name}: clamped angle {resultAngle:F4} rad exceeds " +
            $"cone angle {coneAngle:F4} rad.");
    }

    [Theory]
    [MemberData(nameof(AllRotorConstraints), MathF.PI / 4f)]
    public void Clamp_VariousAxes_AllResultWithinCone(IFabrikConstraint constraint)
    {
        var coneAngle = MathF.PI / 4f; // 45°
        var excessAngle = 80f * MathF.PI / 180f;

        // Test rotations around multiple axes to verify the cone is symmetric.
        var axes = new[]
        {
            Vector3.UnitX,
            Vector3.UnitZ,
            Vector3.Normalize(new Vector3(1f, 0f, 1f)),
            Vector3.Normalize(new Vector3(1f, 0f, -1f)),
            Vector3.Normalize(new Vector3(0f, 1f, 1f)),
        };

        foreach (var axis in axes)
        {
            var orientation = Quaternion.CreateFromAxisAngle(axis, excessAngle);
            var clamped = constraint.Clamp(orientation, ParentIdentity);

            var clampedDirection = Vector3.Transform(Vector3.UnitY, clamped);
            var dot = Math.Clamp(Vector3.Dot(clampedDirection, Vector3.UnitY), -1f, 1f);
            var resultAngle = MathF.Acos(dot);

            Assert.True(resultAngle <= coneAngle + 1e-4f,
                $"{constraint.GetType().Name} axis {axis}: clamped angle " +
                $"{resultAngle:F4} rad exceeds cone angle {coneAngle:F4} rad.");
        }
    }

    // -------------------------------------------------------------------------
    // Clamped orientation is normalised
    // -------------------------------------------------------------------------

    [Theory]
    [MemberData(nameof(AllRotorConstraints), MathF.PI / 4f)]
    public void Clamp_Result_IsNormalised(IFabrikConstraint constraint)
    {
        var orientation = Quaternion.CreateFromAxisAngle(Vector3.UnitX, 80f * MathF.PI / 180f);
        var clamped = constraint.Clamp(orientation, ParentIdentity);
        Assert.True(MathF.Abs(clamped.Length() - 1f) < 1e-5f,
            $"{constraint.GetType().Name}: clamped orientation should be normalised.");
    }

    // -------------------------------------------------------------------------
    // SwingTwistRotorConstraint-specific: twist is preserved
    // -------------------------------------------------------------------------

    [Fact]
    public void SwingTwist_TwistPreservedAfterClamp()
    {
        // Build an orientation that violates the cone with a swing,
        // combined with a meaningful twist around +Y.
        var coneAngle = MathF.PI / 4f;
        var constraint = new SwingTwistRotorConstraint(coneAngle);

        var swing = Quaternion.CreateFromAxisAngle(Vector3.UnitX, 80f * MathF.PI / 180f);
        var twist = Quaternion.CreateFromAxisAngle(Vector3.UnitY, 45f * MathF.PI / 180f);
        var combined = Quaternion.Normalize(swing * twist);

        var clamped = constraint.Clamp(combined, ParentIdentity);

        // Extract the twist component from the clamped result.
        // If twist is preserved, rotating +Y by the clamped orientation and
        // then untwisting should give us back +Y (since swing is on the cone).
        // More directly: project the clamped quaternion's vector part onto +Y
        // to isolate the twist component and check it matches the original twist.
        var originalTwistAngle = 2f * MathF.Acos(Math.Clamp(twist.W, -1f, 1f));

        // Decompose the clamped quaternion into swing and twist.
        var qVec = new Vector3(clamped.X, clamped.Y, clamped.Z);
        var projection = Vector3.Dot(qVec, Vector3.UnitY) * Vector3.UnitY;
        var twistFromClamped = Quaternion.Normalize(
            new Quaternion(projection.X, projection.Y, projection.Z, clamped.W));
        var clampedTwistAngle = 2f * MathF.Acos(Math.Clamp(twistFromClamped.W, -1f, 1f));

        Assert.True(MathF.Abs(clampedTwistAngle - originalTwistAngle) < 1e-4f,
            $"SwingTwist should preserve twist angle. " +
            $"Expected {originalTwistAngle:F4} rad, got {clampedTwistAngle:F4} rad.");
    }

    [Fact]
    public void SwingTwist_PureTwistWithinCone_IsUnchanged()
    {
        // A pure twist (rotation around +Y) should never violate a rotor cone
        // because it produces zero swing. It should pass through unchanged.
        var constraint = new SwingTwistRotorConstraint(MathF.PI / 4f);
        var pureTwist = Quaternion.CreateFromAxisAngle(Vector3.UnitY, 60f * MathF.PI / 180f);
        var clamped = constraint.Clamp(pureTwist, ParentIdentity);

        var diff = Quaternion.Dot(pureTwist, clamped);
        Assert.True(MathF.Abs(diff) > 0.9999f,
            "A pure twist should not be modified by a rotor constraint.");
    }
}
