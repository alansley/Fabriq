using System.Numerics;

namespace Fabriq;

/// <summary>
/// A rotor (symmetric cone) constraint implemented via dot product angle test
/// and spherical linear interpolation (Slerp).
///
/// APPROACH
/// --------
/// Rather than operating in quaternion space, this implementation works with
/// direction vectors. It transforms the bone's rest direction (+Y) by the
/// proposed orientation to get the bone's actual pointing direction, measures
/// the angle between that direction and the parent's pointing direction (+Y),
/// and if that angle exceeds the cone angle, Slerps the orientation back toward
/// identity until the bone direction sits on the cone surface.
///
/// IMPORTANT LIMITATION — TWIST IS DISCARDED
/// ------------------------------------------
/// This implementation operates entirely on the direction the bone points.
/// Twist — rotation around the bone's own long axis — produces no change in
/// the bone's pointing direction and is therefore invisible to this approach.
/// When this constraint clamps an orientation, it adjusts only the directional
/// component and discards any twist that was present.
///
/// This is the most significant limitation of this approach compared to
/// SwingTwistRotorConstraint. Use this implementation when:
///   - You are working in a context where twist is meaningless (e.g. simple
///     2D-style rigs where all bones are in a plane)
///   - You want an implementation that is easy to reason about visually
///   - You are debugging or validating other constraint implementations
///
/// SLERP CLAMPING
/// --------------
/// Slerp from identity toward the current orientation by the ratio
/// (coneAngle / currentAngle) places the result exactly on the cone boundary.
/// This produces a smooth, minimum-rotation correction with no discontinuities.
///
/// CONE ANGLE
/// ----------
/// Valid range: (0, π]. See SwingTwistRotorConstraint for full discussion
/// of the cone angle parameter.
/// </summary>
public sealed class DotProductRotorConstraint : IFabrikConstraint
{
    private float _coneAngleRads;

    /// <inheritdoc/>
    public ConstraintType Type => ConstraintType.Rotor;

    /// <summary>
    /// The maximum permitted angular deviation of the bone's direction from
    /// the parent bone's direction, in radians. Valid range: (0, π].
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
    /// Creates a DotProductRotorConstraint with the given cone angle.
    /// </summary>
    /// <param name="coneAngleRads">
    /// Maximum angular deviation in radians. Must be in the range (0, π].
    /// </param>
    public DotProductRotorConstraint(float coneAngleRads)
    {
        ConeAngleRads = coneAngleRads;
    }

    /// <inheritdoc/>
    /// <remarks>
    /// Uses a dot product to detect whether the bone direction violates the
    /// cone, then projects the direction geometrically onto the cone boundary.
    ///
    /// NOTE ON CLAMPING APPROACH
    /// -------------------------
    /// An earlier implementation used Slerp(identity, orientation, coneAngle/angle)
    /// for clamping. This is incorrect when the rotation mixes swing and twist —
    /// "angle" as measured from the bone direction is the swing angle only, but
    /// Slerp reduces the total quaternion rotation (swing + twist combined).
    /// The result overshoots the cone boundary by a small but real amount.
    ///
    /// Instead, we project the bone direction directly onto the cone rim via
    /// the lateral rejection from the cone axis. This is geometrically exact
    /// regardless of any twist present. Twist is still discarded — see class
    /// summary — but the directional clamping is now accurate.
    /// </remarks>
    public Quaternion Clamp(Quaternion orientation, Quaternion parentOrientation)
    {
        // Transform the bone's rest direction (+Y) into the parent's frame.
        var boneDirection = Vector3.Transform(Vector3.UnitY, orientation);

        // Measure the angular deviation from the parent's rest direction (+Y).
        var dot = Math.Clamp(Vector3.Dot(boneDirection, Vector3.UnitY), -1f, 1f);
        var angle = MathF.Acos(dot);

        // Within the cone — nothing to do.
        if (angle <= _coneAngleRads)
            return orientation;

        // Project the bone direction onto the cone rim.
        // The lateral component (rejection from +Y) gives the circumferential
        // direction on the cone; scaling +Y and lateral by cos/sin of the cone
        // angle gives the nearest point on the cone boundary.
        var lateral = boneDirection - dot * Vector3.UnitY;
        var lateralLength = lateral.Length();

        Vector3 nearestDirection;
        if (lateralLength < 1e-6f)
        {
            // Bone is antiparallel to +Y — any lateral direction is valid.
            var perp = MathF.Abs(Vector3.UnitY.X) < 0.9f
                ? Vector3.Normalize(Vector3.Cross(Vector3.UnitY, Vector3.UnitX))
                : Vector3.Normalize(Vector3.Cross(Vector3.UnitY, Vector3.UnitZ));
            nearestDirection = Vector3.UnitY * MathF.Cos(_coneAngleRads) +
                               perp * MathF.Sin(_coneAngleRads);
        }
        else
        {
            nearestDirection = Vector3.UnitY * MathF.Cos(_coneAngleRads) +
                               (lateral / lateralLength) * MathF.Sin(_coneAngleRads);
        }

        // Compute the shortest-arc quaternion from +Y to the nearest valid direction.
        return RotationFromTo(Vector3.UnitY, nearestDirection);
    }

    private static Quaternion RotationFromTo(Vector3 from, Vector3 to)
    {
        var dot = Vector3.Dot(from, to);
        if (dot > 0.9999f) return Quaternion.Identity;
        if (dot < -0.9999f)
        {
            var perp = MathF.Abs(from.X) < 0.9f
                ? Vector3.Cross(from, Vector3.UnitX)
                : Vector3.Cross(from, Vector3.UnitZ);
            return Quaternion.CreateFromAxisAngle(Vector3.Normalize(perp), MathF.PI);
        }
        var axis = Vector3.Cross(from, to);
        return Quaternion.Normalize(new Quaternion(axis.X, axis.Y, axis.Z, 1f + dot));
    }
}
