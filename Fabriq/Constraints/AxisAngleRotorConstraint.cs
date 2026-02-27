using System.Numerics;

namespace Fabriq;

/// <summary>
/// A rotor (symmetric cone) constraint implemented via axis-angle clamping.
///
/// APPROACH
/// --------
/// Treats the orientation quaternion directly as an axis-angle rotation.
/// Extracts the rotation angle, compares it against the cone angle, and if
/// exceeded, clamps the angle while preserving the rotation axis.
///
/// IMPORTANT LIMITATION — TWIST IS NOT PRESERVED
/// ----------------------------------------------
/// This implementation does not separate swing from twist. The rotation angle
/// extracted from the quaternion combines both directional deviation (swing)
/// and axial rotation (twist) into a single value. Clamping the angle will
/// therefore reduce twist as well as swing — the two are indistinguishable
/// from this approach alone.
///
/// If your rig uses twist constraints (ITwistConstraint) and accurate twist
/// preservation matters, use SwingTwistRotorConstraint instead. Use this
/// implementation when:
///   - Twist is not relevant to your use case
///   - You want the simplest possible rotor constraint
///   - Performance is critical and the twist error is acceptable
///
/// KNOWN SINGULARITIES
/// -------------------
/// Axis-angle representation is singular at 0° and 360°. Near these angles,
/// the rotation axis becomes numerically unstable. This implementation guards
/// against degenerate axes and returns the orientation unchanged when the axis
/// cannot be determined reliably.
///
/// CONE ANGLE
/// ----------
/// Valid range: (0, π]. See SwingTwistRotorConstraint for full discussion
/// of the cone angle parameter.
/// </summary>
public sealed class AxisAngleRotorConstraint : IFabrikConstraint
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
    /// Creates an AxisAngleRotorConstraint with the given cone angle.
    /// </summary>
    /// <param name="coneAngleRads">
    /// Maximum rotation angle in radians. Must be in the range (0, π].
    /// </param>
    public AxisAngleRotorConstraint(float coneAngleRads)
    {
        ConeAngleRads = coneAngleRads;
    }

    /// <inheritdoc/>
    /// <remarks>
    /// Extracts the axis-angle representation of the orientation, clamps the
    /// angle to the cone angle, and reconstructs the quaternion. Twist is not
    /// preserved — see class summary.
    /// </remarks>
    public Quaternion Clamp(Quaternion orientation, Quaternion parentOrientation)
    {
        // Extract the rotation angle from the quaternion.
        // q.W = cos(θ/2), so θ = 2 * acos(q.W).
        var angle = 2f * MathF.Acos(Math.Clamp(orientation.W, -1f, 1f));

        // Within the cone — nothing to do.
        if (angle <= _coneAngleRads)
            return orientation;

        // Extract the rotation axis from the vector part of the quaternion.
        var axis = new Vector3(orientation.X, orientation.Y, orientation.Z);
        var axisLength = axis.Length();

        // Degenerate axis — cannot determine a meaningful clamping direction.
        if (axisLength < 1e-6f)
            return orientation;

        // Clamp the angle to the cone boundary, preserving the rotation axis.
        return Quaternion.CreateFromAxisAngle(axis / axisLength, _coneAngleRads);
    }
}
