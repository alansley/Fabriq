using System.Numerics;

namespace Fabriq;

/// <summary>
/// The type of rotational constraint applied to a bone.
/// </summary>
public enum ConstraintType
{
    /// <summary>
    /// A symmetric cone of allowed orientations around the parent bone's axis.
    /// Equal freedom in all directions. Carried over from Caliko.
    /// </summary>
    Rotor,

    /// <summary>
    /// Rotation restricted to a single axis with a minimum and maximum angle.
    /// No lateral deviation permitted. Carried over from Caliko.
    /// </summary>
    Hinge,

    /// <summary>
    /// An elliptical cone of allowed orientations, with independently configurable
    /// freedom in the horizontal (X) and vertical (Y) directions relative to the
    /// parent bone's orientation. New in Fabriq.
    /// </summary>
    EllipticalCone,
}

/// <summary>
/// Defines a rotational constraint on a bone relative to its parent.
///
/// A constraint is responsible for clamping a proposed orientation back into its
/// valid region. This allows new constraint types to be added without modifying
/// the solver — the solver simply calls <see cref="Clamp"/> after each pass.
///
/// All constraint types operate in the reference frame of the parent bone's
/// current orientation. See docs/constraint-model.md for full details.
/// </summary>
public interface IFabrikConstraint
{
    /// <summary>
    /// The type of this constraint.
    /// </summary>
    ConstraintType Type { get; }

    /// <summary>
    /// Clamps the proposed orientation to the nearest valid orientation within
    /// this constraint's allowed region, given the parent bone's orientation
    /// as the reference frame.
    /// </summary>
    /// <param name="orientation">The proposed orientation to clamp.</param>
    /// <param name="parentOrientation">
    /// The parent bone's current orientation, used as the reference frame.
    /// This includes any twist the parent has applied.
    /// </param>
    /// <returns>The nearest valid orientation within this constraint.</returns>
    Quaternion Clamp(Quaternion orientation, Quaternion parentOrientation);
}
