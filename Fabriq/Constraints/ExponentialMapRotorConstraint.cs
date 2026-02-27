using System.Numerics;

namespace Fabriq;

/// <summary>
/// A rotor (symmetric cone) constraint implemented via the exponential map
/// (tangent space) representation of rotations.
///
/// APPROACH
/// --------
/// The exponential map represents a rotation quaternion as a vector in the
/// tangent space of SO(3) — the space of all 3D rotations. For a quaternion
/// q = (cos(θ/2), sin(θ/2) * n̂), the exponential map (logarithm of q) is:
///
///   log(q) = (θ/2) * n̂
///
/// This is simply a vector whose direction is the rotation axis and whose
/// magnitude is the half-angle of rotation. Constraining the rotation to a
/// cone of angle α is then equivalent to clamping the magnitude of this vector
/// to α/2 — a simple vector length clamp in tangent space.
///
/// The clamped vector is then mapped back to a quaternion via the exponential:
///
///   exp(v) = (cos(|v|), sin(|v|) * normalize(v))
///
/// TWIST BEHAVIOUR
/// ---------------
/// Unlike SwingTwistRotorConstraint, this implementation does not explicitly
/// separate swing from twist before clamping. The vector magnitude in tangent
/// space represents the total rotation angle, which combines both swing and
/// twist. Clamping the magnitude therefore reduces both in proportion — twist
/// is attenuated rather than preserved.
///
/// For rigs where accurate twist preservation matters, use
/// SwingTwistRotorConstraint. This implementation is appropriate when:
///   - You want a mathematically clean, tangent-space approach
///   - The rig does not use twist constraints
///   - You are exploring or comparing constraint methods
///   - Slightly attenuated twist is acceptable for your use case
///
/// NUMERICAL PROPERTIES
/// --------------------
/// The exponential map is well-behaved near identity (small rotations) and
/// avoids the singularities of axis-angle representation at 0° and 360°.
/// It is the basis of many motion interpolation and blending systems in
/// high-end animation software.
///
/// CONE ANGLE
/// ----------
/// Valid range: (0, π]. See SwingTwistRotorConstraint for full discussion
/// of the cone angle parameter.
/// </summary>
public sealed class ExponentialMapRotorConstraint : IFabrikConstraint
{
    private float _coneAngleRads;

    /// <inheritdoc/>
    public ConstraintType Type => ConstraintType.Rotor;

    /// <summary>
    /// The maximum permitted rotation angle from the parent bone's direction,
    /// in radians. Valid range: (0, π].
    /// </summary>
    public float ConeAngleRads
    {
        get => _coneAngleRads;
        set
        {
            if (value <= 0f || value > MathF.PI)
                throw new ArgumentOutOfRangeException(nameof(value),
                    $"Cone angle must be in the range (0, π]. Received: {value}");
            _coneAngleRads = value;
        }
    }

    /// <summary>
    /// Creates an ExponentialMapRotorConstraint with the given cone angle.
    /// </summary>
    /// <param name="coneAngleRads">
    /// Maximum rotation angle in radians. Must be in the range (0, π].
    /// </param>
    public ExponentialMapRotorConstraint(float coneAngleRads)
    {
        ConeAngleRads = coneAngleRads;
    }

    /// <inheritdoc/>
    /// <remarks>
    /// Maps the orientation to tangent space via the quaternion logarithm,
    /// clamps the vector magnitude to the half cone angle, then maps back via
    /// the quaternion exponential. Twist is attenuated proportionally rather
    /// than preserved — see class summary.
    /// </remarks>
    public Quaternion Clamp(Quaternion orientation, Quaternion parentOrientation)
    {
        var xyz = new Vector3(orientation.X, orientation.Y, orientation.Z);
        var xyzLength = xyz.Length();

        // Near-identity rotation — already within any reasonable cone.
        if (xyzLength < 1e-6f)
            return orientation;

        // Compute the half-angle (the magnitude of the log-map vector).
        // q.W = cos(θ/2), so halfAngle = acos(q.W).
        var halfAngle = MathF.Acos(Math.Clamp(orientation.W, -1f, 1f));
        var fullAngle = 2f * halfAngle;

        // Within the cone — nothing to do.
        if (fullAngle <= _coneAngleRads)
            return orientation;

        // Clamp to the cone boundary in tangent space.
        var clampedHalfAngle = _coneAngleRads / 2f;
        var axis = xyz / xyzLength;

        // Map back via the quaternion exponential: exp(θ/2 * n̂).
        return Quaternion.Normalize(new Quaternion(
            axis * MathF.Sin(clampedHalfAngle),
            MathF.Cos(clampedHalfAngle)));
    }
}
