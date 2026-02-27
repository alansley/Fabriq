using System.Numerics;

namespace Fabriq;

/// <summary>
/// Concrete implementation of <see cref="IFabrikBone"/>.
///
/// A bone is the fundamental building block of a FABRIK chain. It represents
/// a rigid segment defined by an orientation relative to its parent and a fixed
/// length. It carries optional constraints that the solver enforces after each
/// pass, and an optional name for identification and debugging.
///
/// BONE DIRECTION CONVENTION
/// -------------------------
/// A bone with identity orientation (Quaternion.Identity) points in the local
/// +Y direction. All orientations are expressed as rotations from this default.
///
/// This convention is deliberate. In character animation and anatomy, bones
/// naturally "grow" along an axis — a finger extends from knuckle to tip, a
/// spine segment extends upward. Choosing +Y as the rest direction means an
/// unrotated bone points upward, which matches the mental model most naturally
/// when building rigs from scratch.
///
/// Factory methods such as <see cref="FromDirection"/> and <see cref="For2D"/>
/// handle the conversion from human-friendly direction vectors and angles into
/// the underlying quaternion representation, so callers rarely need to think
/// about this convention directly.
///
/// ORIENTATION NORMALISATION
/// -------------------------
/// Quaternions must be unit-length to correctly represent rotations. Setting
/// <see cref="Orientation"/> automatically normalises the provided value.
/// This silently corrects minor floating-point drift that accumulates over
/// many solve iterations, rather than allowing it to compound into visible
/// artefacts. If you set an orientation of approximately zero length, an
/// <see cref="ArgumentException"/> is thrown — that is not a valid rotation.
///
/// LENGTH VALIDATION
/// -----------------
/// A bone must have a positive length. Zero or negative lengths are physically
/// meaningless and will cause division-by-zero in the solver. Setting
/// <see cref="Length"/> to a non-positive value throws an
/// <see cref="ArgumentOutOfRangeException"/> immediately rather than allowing
/// a corrupt chain to be built silently.
///
/// CONSTRAINTS
/// -----------
/// <see cref="Constraint"/> governs the valid range of orientations for this
/// bone relative to its parent (rotor, hinge, or elliptical cone).
/// <see cref="TwistConstraint"/> governs rotation around the bone's own long
/// axis independently. Both are null by default — an unconstrained bone.
/// See <see cref="IFabrikConstraint"/> and <see cref="ITwistConstraint"/> for
/// full constraint documentation, and docs/constraint-model.md for the design
/// rationale.
/// </summary>
public class FabrikBone : IFabrikBone
{
    private Quaternion _orientation;
    private float _length;

    // -------------------------------------------------------------------------
    // Properties
    // -------------------------------------------------------------------------

    /// <inheritdoc/>
    /// <remarks>
    /// Automatically normalised on set. An orientation of near-zero magnitude
    /// is rejected with an <see cref="ArgumentException"/>.
    /// </remarks>
    public Quaternion Orientation
    {
        get => _orientation;
        set
        {
            if (value.LengthSquared() < 1e-10f)
                throw new ArgumentException(
                    "Orientation quaternion has near-zero magnitude and cannot " +
                    "represent a valid rotation.", nameof(value));

            _orientation = Quaternion.Normalize(value);
        }
    }

    /// <inheritdoc/>
    /// <remarks>
    /// Must be greater than zero. Throws <see cref="ArgumentOutOfRangeException"/>
    /// if a non-positive value is supplied.
    /// </remarks>
    public float Length
    {
        get => _length;
        set
        {
            if (value <= 0f)
                throw new ArgumentOutOfRangeException(nameof(value),
                    $"Bone length must be greater than zero. Received: {value}");

            _length = value;
        }
    }

    /// <inheritdoc/>
    public IFabrikConstraint? Constraint { get; set; }

    /// <inheritdoc/>
    public ITwistConstraint? TwistConstraint { get; set; }

    /// <inheritdoc/>
    public string? Name { get; set; }

    // -------------------------------------------------------------------------
    // Constructors
    // -------------------------------------------------------------------------

    /// <summary>
    /// Creates a bone pointing in the +Y direction (identity orientation)
    /// with the given length.
    /// </summary>
    /// <param name="length">The length of the bone. Must be greater than zero.</param>
    public FabrikBone(float length)
    {
        Length = length;
        _orientation = Quaternion.Identity;
    }

    /// <summary>
    /// Creates a bone with an explicit orientation and length.
    /// The orientation is normalised automatically.
    /// </summary>
    /// <param name="orientation">
    /// The initial orientation of the bone in local space, relative to its parent.
    /// Will be normalised automatically.
    /// </param>
    /// <param name="length">The length of the bone. Must be greater than zero.</param>
    public FabrikBone(Quaternion orientation, float length)
    {
        Orientation = orientation;
        Length = length;
    }

    /// <summary>
    /// Creates a bone with an explicit orientation, length, and name.
    /// The orientation is normalised automatically.
    /// </summary>
    /// <param name="orientation">
    /// The initial orientation of the bone in local space, relative to its parent.
    /// Will be normalised automatically.
    /// </param>
    /// <param name="length">The length of the bone. Must be greater than zero.</param>
    /// <param name="name">An optional name for identification and debugging.</param>
    public FabrikBone(Quaternion orientation, float length, string? name)
    {
        Orientation = orientation;
        Length = length;
        Name = name;
    }

    /// <summary>
    /// Copy constructor. Creates a deep copy of the given bone, including
    /// its orientation, length, name, and constraint references.
    /// </summary>
    /// <remarks>
    /// Constraint references are shallow-copied — the new bone shares the same
    /// constraint instances as the original. This is intentional: constraints
    /// are immutable descriptors of allowed motion, not per-bone state, so
    /// sharing them is both safe and efficient.
    /// </remarks>
    /// <param name="other">The bone to copy.</param>
    public FabrikBone(FabrikBone other)
    {
        ArgumentNullException.ThrowIfNull(other);
        _orientation = other._orientation;
        _length = other._length;
        Name = other.Name;
        Constraint = other.Constraint;
        TwistConstraint = other.TwistConstraint;
    }

    // -------------------------------------------------------------------------
    // Factory methods
    // -------------------------------------------------------------------------

    /// <summary>
    /// Creates a bone oriented to point in the given world-space direction,
    /// with the given length.
    ///
    /// The direction vector is normalised automatically. The resulting bone's
    /// local +Y axis will point in the given direction relative to its parent's
    /// frame of reference.
    /// </summary>
    /// <param name="direction">
    /// The direction the bone should point. Must not be a zero vector.
    /// </param>
    /// <param name="length">The length of the bone. Must be greater than zero.</param>
    /// <returns>A new <see cref="FabrikBone"/> pointing in the given direction.</returns>
    public static FabrikBone FromDirection(Vector3 direction, float length)
    {
        if (direction.LengthSquared() < 1e-10f)
            throw new ArgumentException(
                "Direction vector has near-zero magnitude and cannot define a " +
                "bone orientation.", nameof(direction));

        var orientation = RotationFromTo(Vector3.UnitY, Vector3.Normalize(direction));
        return new FabrikBone(orientation, length);
    }

    /// <summary>
    /// Creates a bone constrained to the XY plane (Z = 0), oriented at the
    /// given angle from the positive X axis, with the given length.
    ///
    /// This is the preferred factory for 2D use cases. Fabriq operates in 3D
    /// space — 2D is the degenerate case where Z is held at zero. The resulting
    /// bone lies entirely in the XY plane with no Z component.
    /// </summary>
    /// <param name="angleRads">
    /// The angle in radians from the positive X axis, measured anticlockwise
    /// in the XY plane. Use <see cref="MathF.PI"/> multiples for common angles
    /// (e.g. MathF.PI / 2f for 90°, pointing along +Y).
    /// </param>
    /// <param name="length">The length of the bone. Must be greater than zero.</param>
    /// <returns>A new <see cref="FabrikBone"/> lying in the XY plane.</returns>
    public static FabrikBone For2D(float angleRads, float length)
    {
        var direction = new Vector3(MathF.Cos(angleRads), MathF.Sin(angleRads), 0f);
        return FromDirection(direction, length);
    }

    // -------------------------------------------------------------------------
    // Object overrides
    // -------------------------------------------------------------------------

    /// <summary>
    /// Returns a human-readable description of this bone, including its name
    /// (if set), orientation, and length. Useful for debugging.
    /// </summary>
    public override string ToString()
    {
        var nameStr = Name is not null ? $"\"{Name}\" " : string.Empty;
        return $"FabrikBone {nameStr}{{ " +
               $"Orientation: {_orientation}, " +
               $"Length: {_length:F4}, " +
               $"Constraint: {Constraint?.Type.ToString() ?? "None"}, " +
               $"Twist: {(TwistConstraint is not null ? $"[{TwistConstraint.MinAngleRads:F3}..{TwistConstraint.MaxAngleRads:F3}] rads" : "None")}" +
               $" }}";
    }

    /// <inheritdoc/>
    public override bool Equals(object? obj)
    {
        if (obj is not FabrikBone other) return false;
        return _orientation == other._orientation &&
               MathF.Abs(_length - other._length) < 1e-6f &&
               Name == other.Name &&
               Constraint == other.Constraint &&
               TwistConstraint == other.TwistConstraint;
    }

    /// <inheritdoc/>
    public override int GetHashCode() =>
        HashCode.Combine(_orientation, _length, Name, Constraint, TwistConstraint);

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------

    /// <summary>
    /// Computes the shortest-arc quaternion rotation from one unit vector to
    /// another. Handles the degenerate cases where the vectors are nearly
    /// parallel (returns identity) or nearly antiparallel (rotates 180° around
    /// a perpendicular axis).
    /// </summary>
    private static Quaternion RotationFromTo(Vector3 from, Vector3 to)
    {
        var dot = Vector3.Dot(from, to);

        // Vectors are nearly identical — no rotation needed.
        if (dot > 0.9999f)
            return Quaternion.Identity;

        // Vectors are nearly opposite — any perpendicular axis will do for the
        // 180° rotation. We pick the one least parallel to 'from' to avoid
        // numerical instability in the cross product.
        if (dot < -0.9999f)
        {
            var perp = MathF.Abs(from.X) < 0.9f
                ? Vector3.Cross(from, Vector3.UnitX)
                : Vector3.Cross(from, Vector3.UnitZ);
            return Quaternion.CreateFromAxisAngle(Vector3.Normalize(perp), MathF.PI);
        }

        // General case: axis is the cross product, w is derived from the dot
        // product. Normalise to correct any floating-point error.
        var axis = Vector3.Cross(from, to);
        var w = 1.0f + dot;
        return Quaternion.Normalize(new Quaternion(axis.X, axis.Y, axis.Z, w));
    }
}
