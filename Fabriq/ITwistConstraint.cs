namespace Fabriq;

/// <summary>
/// Defines a constraint on rotation around a bone's own long axis (twist).
///
/// "Twist" is the correct term here — it refers specifically to the twist component
/// of a swing-twist quaternion decomposition, which separates a bone's orientation
/// into its directional component (swing) and its axial rotation component (twist).
///
/// Twist is orthogonal to the other constraint types — a bone may have both an
/// <see cref="IFabrikConstraint"/> restricting its directional range and an
/// <see cref="ITwistConstraint"/> restricting how far it rotates around its own
/// axis. The two are applied independently.
///
/// Twist is measured relative to the parent bone's orientation, not world space
/// and not the chain root. This ensures the valid twist range travels correctly
/// with the chain — raising your upper arm does not change the neutral position
/// of your forearm's twist.
///
/// Twist propagates down the chain automatically. Because all constraints are
/// evaluated against the parent bone's complete quaternion orientation, any twist
/// applied to a bone shifts the reference frame for every downstream bone without
/// any special handling required.
///
/// Values are stored in radians. Use <see cref="MathF.PI"/> and its multiples
/// for convenient construction (e.g. MathF.PI / 4f for 45°, MathF.PI / 2f for 90°).
///
/// Example ranges (illustrative, not enforced):
///   Forearm pronation/supination : -1.396f to +1.396f  (~80°)
///   Robot tool head              : -MathF.PI to +MathF.PI (unconstrained spin)
///   Finger                       : near zero
/// </summary>
public interface ITwistConstraint
{
    /// <summary>
    /// The minimum permitted twist angle in radians, measured relative to the
    /// parent bone's orientation around this bone's own long axis.
    /// </summary>
    float MinAngleRads { get; }

    /// <summary>
    /// The maximum permitted twist angle in radians, measured relative to the
    /// parent bone's orientation around this bone's own long axis.
    /// </summary>
    float MaxAngleRads { get; }

    /// <summary>
    /// Clamps the proposed twist angle to the nearest valid angle within
    /// [<see cref="MinAngleRads"/>, <see cref="MaxAngleRads"/>].
    /// </summary>
    /// <param name="twistAngleRads">The proposed twist angle in radians.</param>
    /// <returns>The clamped twist angle in radians.</returns>
    float Clamp(float twistAngleRads);
}
