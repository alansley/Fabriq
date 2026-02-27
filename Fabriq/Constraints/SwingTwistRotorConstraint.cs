using System.Numerics;

namespace Fabriq;

/// <summary>
/// A rotor (symmetric cone) constraint implemented via swing-twist decomposition.
///
/// This is the PREFERRED rotor constraint implementation in Fabriq. It should
/// be your default choice unless you have a specific reason to use another.
///
/// HOW IT WORKS
/// ------------
/// Any quaternion orientation can be decomposed into two components:
///
///   Swing  — the rotational deviation of the bone's direction from its parent.
///            This is the component that the rotor constraint limits.
///
///   Twist  — rotation of the bone around its own long axis (+Y in local space).
///            This is handled separately by ITwistConstraint and is deliberately
///            left untouched by this constraint.
///
/// The decomposition proceeds as follows:
///
///   1. Project the orientation's vector part onto the twist axis (+Y).
///   2. The projection gives the twist component; the remainder is the swing.
///   3. Measure the swing angle.
///   4. If the swing angle exceeds the cone angle, clamp the swing back to the
///      cone surface while leaving the twist component completely unchanged.
///   5. Reconstruct the final orientation as clampedSwing * twist.
///
/// WHY THIS IS PREFERRED
/// ----------------------
/// The other rotor implementations (axis-angle, dot product, exponential map,
/// CGA) either discard twist information or operate on direction vectors rather
/// than quaternions. Swing-twist decomposition is the only approach that:
///
///   - Works natively in quaternion space throughout
///   - Preserves twist independently of the directional constraint
///   - Aligns naturally with the separate ITwistConstraint model
///   - Produces smooth, artefact-free clamping without sudden discontinuities
///
/// CONE ANGLE
/// ----------
/// The cone angle is the maximum permitted swing deviation in radians. It is
/// symmetric — the same limit applies in all directions around the parent axis.
/// For asymmetric limits, use EllipticalConeConstraint instead.
///
/// Valid range: (0, π]. A cone angle of π means the bone can swing in any
/// direction (full hemisphere). A cone angle approaching 0 means the bone
/// is nearly locked to its parent direction.
/// </summary>
public sealed class SwingTwistRotorConstraint : IFabrikConstraint
{
    private float _coneAngleRads;

    /// <inheritdoc/>
    public ConstraintType Type => ConstraintType.Rotor;

    /// <summary>
    /// The maximum permitted swing deviation from the parent bone's direction,
    /// in radians. Symmetric in all directions around the parent axis.
    /// Valid range: (0, π].
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
    /// Creates a SwingTwistRotorConstraint with the given cone angle.
    /// </summary>
    /// <param name="coneAngleRads">
    /// Maximum swing deviation in radians. Must be in the range (0, π].
    /// </param>
    public SwingTwistRotorConstraint(float coneAngleRads)
    {
        ConeAngleRads = coneAngleRads;
    }

    /// <inheritdoc/>
    /// <remarks>
    /// Decomposes the orientation into swing and twist components, clamps the
    /// swing to the cone surface if it exceeds the cone angle, and reconstructs
    /// the orientation with the twist component preserved exactly.
    /// </remarks>
    public Quaternion Clamp(Quaternion orientation, Quaternion parentOrientation)
    {
        var (swing, twist) = DecomposeSwingTwist(orientation, Vector3.UnitY);

        // Measure the swing angle — the full rotation angle of the swing component.
        var swingAngle = 2f * MathF.Acos(Math.Clamp(swing.W, -1f, 1f));

        // Within the cone — nothing to do.
        if (swingAngle <= _coneAngleRads)
            return orientation;

        // Clamp swing back to the cone surface.
        var swingAxis = new Vector3(swing.X, swing.Y, swing.Z);
        var axisLength = swingAxis.Length();

        // If the swing axis is degenerate, the bone is already near-identity —
        // no meaningful clamping is possible.
        if (axisLength < 1e-6f)
            return orientation;

        var clampedSwing = Quaternion.CreateFromAxisAngle(
            swingAxis / axisLength, _coneAngleRads);

        // Reconstruct: clampedSwing brings the bone direction within the cone;
        // twist is reapplied unchanged so axial rotation is preserved.
        return Quaternion.Normalize(clampedSwing * twist);
    }

    // -------------------------------------------------------------------------
    // Swing-twist decomposition
    // -------------------------------------------------------------------------

    /// <summary>
    /// Decomposes a quaternion into swing and twist components relative to the
    /// given twist axis.
    ///
    /// Swing is the component that rotates the bone's direction (perpendicular
    /// to the twist axis). Twist is the component that rotates around the twist
    /// axis itself. The original quaternion equals swing * twist.
    /// </summary>
    /// <param name="q">The quaternion to decompose. Need not be normalised.</param>
    /// <param name="twistAxis">
    /// The axis around which twist is defined. For Fabriq bones this is
    /// Vector3.UnitY — the bone's local pointing direction at rest.
    /// </param>
    private static (Quaternion swing, Quaternion twist) DecomposeSwingTwist(
        Quaternion q, Vector3 twistAxis)
    {
        // Project the quaternion's vector part onto the twist axis.
        // This projection is the vector part of the twist quaternion.
        var qVec = new Vector3(q.X, q.Y, q.Z);
        var projection = Vector3.Dot(qVec, twistAxis) * twistAxis;

        var twist = Quaternion.Normalize(
            new Quaternion(projection.X, projection.Y, projection.Z, q.W));

        // Swing is whatever is left after removing the twist.
        var swing = Quaternion.Normalize(q * Quaternion.Conjugate(twist));

        return (swing, twist);
    }
}
