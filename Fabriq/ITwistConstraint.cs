namespace Fabriq;

/// <summary>
/// Defines a constraint on rotation around a bone's own long axis (twist).
///
/// Twist is orthogonal to the other constraint types — a bone may have both an
/// <see cref="IFabrikConstraint"/> restricting its directional range and an
/// <see cref="ITwistConstraint"/> restricting how far it rotates around its own
/// axis. The two are applied independently.
///
/// Twist propagates down the chain automatically. Because all constraints are
/// evaluated against the parent bone's complete quaternion orientation, any twist
/// applied to a bone shifts the reference frame for every downstream bone without
/// any special handling required.
///
/// Example ranges (illustrative, not enforced):
///   Forearm pronation/supination : -80° to +80°
///   Robot tool head              : -180° to +180° (or use Unconstrained)
///   Finger                       : negligible, near 0°
/// </summary>
public interface ITwistConstraint
{
    /// <summary>
    /// The minimum permitted twist angle in degrees, measured from the bone's
    /// neutral reference orientation around its own long axis.
    /// </summary>
    float MinAngleDegrees { get; }

    /// <summary>
    /// The maximum permitted twist angle in degrees, measured from the bone's
    /// neutral reference orientation around its own long axis.
    /// </summary>
    float MaxAngleDegrees { get; }

    /// <summary>
    /// Clamps the proposed twist angle to the nearest valid angle within
    /// [<see cref="MinAngleDegrees"/>, <see cref="MaxAngleDegrees"/>].
    /// </summary>
    /// <param name="twistAngleDegrees">The proposed twist angle in degrees.</param>
    /// <returns>The clamped twist angle in degrees.</returns>
    float Clamp(float twistAngleDegrees);
}
