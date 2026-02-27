using System.Numerics;

namespace Fabriq;

/// <summary>
/// A rotor (symmetric cone) constraint implemented using principles from
/// Conformal Geometric Algebra (CGA).
///
/// BACKGROUND
/// ----------
/// Conformal Geometric Algebra (CGA) is a mathematical framework that unifies
/// rotations, translations, and reflections into a single algebraic structure.
/// The FABRIK paper (Aristidou and Lasenby, 2011) mentions CGA as an avenue
/// for optimisation and more elegant constraint representation (refs [25,26] in
/// the paper: Hestenes and Sobczyk, Doran and Lasenby).
///
/// In CGA, rotations are represented as rotors — elements of the even
/// sub-algebra — and constraints can be expressed geometrically as
/// intersections of spheres and planes rather than as angle comparisons.
/// The key insight is that the constraint boundary (a cone) can be described
/// as the intersection of two conformal spheres, and clamping is achieved by
/// projecting the current direction onto this intersection — the "meet"
/// operation in CGA.
///
/// THIS IMPLEMENTATION
/// -------------------
/// A full CGA implementation requires a complete geometric algebra library.
/// Rather than introducing that dependency, this implementation encodes the
/// geometric core of the CGA approach using System.Numerics types, applying
/// the sphere-intersection concept that CGA formalises elegantly.
///
/// The algorithm:
///   1. Transform the bone's rest direction (+Y) to get its current pointing
///      direction in parent space.
///   2. Project this direction onto the unit sphere.
///   3. Find the nearest point on the cone boundary (the intersection of the
///      unit sphere with the cone's limiting plane) using the CGA meet concept
///      — geometrically, this is the point on the cone rim closest to the
///      current direction, found by normalising the rejection of the bone
///      direction from the cone axis and rotating it to the cone angle.
///   4. Compute the quaternion that rotates from the parent's rest direction
///      to this nearest valid direction.
///
/// The result is a constrained orientation that lies exactly on the cone
/// boundary via a geometrically exact sphere projection — as opposed to the
/// Slerp interpolation used by DotProductRotorConstraint, which follows an
/// arc rather than a direct sphere projection.
///
/// TWIST BEHAVIOUR
/// ---------------
/// Like DotProductRotorConstraint, this implementation operates on direction
/// vectors and does not preserve twist. The sphere projection finds the nearest
/// valid pointing direction, but the axial rotation (twist) of the bone around
/// that direction is not maintained. Use SwingTwistRotorConstraint when twist
/// preservation is required.
///
/// WHEN TO USE THIS
/// ----------------
/// The CGA approach produces a geometrically exact nearest-point on the cone
/// boundary, which can differ subtly from the Slerp interpolation of
/// DotProductRotorConstraint and the swing decomposition of
/// SwingTwistRotorConstraint. Use this when:
///   - Geometric exactness of the constraint boundary projection matters
///   - You are comparing CGA-inspired results against other approaches
///   - Twist is not relevant to your use case
///   - You are interested in exploring the CGA formulation for research purposes
///
/// CONE ANGLE
/// ----------
/// Valid range: (0, π]. See SwingTwistRotorConstraint for full discussion
/// of the cone angle parameter.
/// </summary>
public sealed class CGARotorConstraint : IFabrikConstraint
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
    /// Creates a CGARotorConstraint with the given cone angle.
    /// </summary>
    /// <param name="coneAngleRads">
    /// Maximum angular deviation in radians. Must be in the range (0, π].
    /// </param>
    public CGARotorConstraint(float coneAngleRads)
    {
        ConeAngleRads = coneAngleRads;
    }

    /// <inheritdoc/>
    /// <remarks>
    /// Applies the CGA sphere-intersection approach to find the nearest valid
    /// direction on the cone boundary. See class summary for full details.
    /// Twist is not preserved.
    /// </remarks>
    public Quaternion Clamp(Quaternion orientation, Quaternion parentOrientation)
    {
        // Get the bone's current pointing direction in parent space.
        var boneDirection = Vector3.Transform(Vector3.UnitY, orientation);

        // Measure the angular deviation from the parent's rest direction (+Y).
        var dot = Math.Clamp(Vector3.Dot(boneDirection, Vector3.UnitY), -1f, 1f);
        var angle = MathF.Acos(dot);

        // Within the cone — nothing to do.
        if (angle <= _coneAngleRads)
            return orientation;

        // CGA SPHERE PROJECTION
        // ---------------------
        // The cone boundary is the set of all unit vectors at exactly
        // _coneAngleRads from the cone axis (+Y). The nearest point on this
        // boundary to the current bone direction is found by:
        //
        //   1. Computing the rejection of boneDirection from the cone axis —
        //      this is the lateral component of boneDirection (the part
        //      perpendicular to +Y), which defines the circumferential
        //      direction on the cone.
        //
        //   2. Normalising the rejection to get the unit lateral direction.
        //      This is the same circumferential position on the cone rim as
        //      the current bone direction — the CGA "meet" of the direction
        //      sphere with the cone's axial plane.
        //
        //   3. Constructing the nearest valid direction by blending the cone
        //      axis and the lateral direction at the cone angle.

        // Lateral component of boneDirection (rejection from +Y axis).
        var lateral = boneDirection - Vector3.Dot(boneDirection, Vector3.UnitY) * Vector3.UnitY;
        var lateralLength = lateral.Length();

        // Degenerate case: boneDirection is antiparallel to +Y — any lateral
        // direction is equally valid. Pick an arbitrary perpendicular.
        Vector3 lateralUnit;
        if (lateralLength < 1e-6f)
        {
            lateralUnit = MathF.Abs(Vector3.UnitY.X) < 0.9f
                ? Vector3.Normalize(Vector3.Cross(Vector3.UnitY, Vector3.UnitX))
                : Vector3.Normalize(Vector3.Cross(Vector3.UnitY, Vector3.UnitZ));
        }
        else
        {
            lateralUnit = lateral / lateralLength;
        }

        // The nearest valid direction lies on the cone rim at _coneAngleRads
        // from +Y, in the same circumferential direction as boneDirection.
        var nearestValidDirection =
            Vector3.UnitY * MathF.Cos(_coneAngleRads) +
            lateralUnit * MathF.Sin(_coneAngleRads);

        // Compute the shortest-arc quaternion from the rest direction (+Y) to
        // the nearest valid direction — this is the clamped orientation.
        return RotationFromTo(Vector3.UnitY, Vector3.Normalize(nearestValidDirection));
    }

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------

    /// <summary>
    /// Computes the shortest-arc quaternion rotation from one unit vector to
    /// another. Handles parallel and antiparallel edge cases.
    /// </summary>
    private static Quaternion RotationFromTo(Vector3 from, Vector3 to)
    {
        var dot = Vector3.Dot(from, to);

        if (dot > 0.9999f)
            return Quaternion.Identity;

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
