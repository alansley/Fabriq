using System.Numerics;

namespace Fabriq;

/// <summary>
/// A single bone in a FABRIK chain.
///
/// A bone is defined by its orientation relative to its parent and its length.
/// It has no knowledge of its position in world space — that is the chain's
/// responsibility. World-space positions are derived by traversing the chain
/// from the base, not stored on individual bones.
///
/// Constraints are evaluated relative to the parent bone's full quaternion
/// orientation, including any twist that parent has applied. This means
/// constraints propagate naturally down the chain without any special handling.
/// </summary>
public interface IFabrikBone
{
    /// <summary>
    /// The orientation of this bone in local space, relative to its parent bone's
    /// orientation. An identity quaternion means the bone points in the same
    /// direction as its parent.
    /// </summary>
    Quaternion Orientation { get; set; }

    /// <summary>
    /// The length of this bone. Must be greater than zero.
    /// </summary>
    float Length { get; set; }

    /// <summary>
    /// The constraint defining the valid range of orientations for this bone relative
    /// to its parent. When null, the bone is unconstrained and may adopt any orientation.
    /// See <see cref="IFabrikConstraint"/> for available constraint types.
    /// </summary>
    IFabrikConstraint? Constraint { get; set; }

    /// <summary>
    /// The twist constraint defining the valid range of rotation around this bone's
    /// own long axis. When null, twist is unconstrained.
    ///
    /// Twist propagates down the chain — any twist applied to this bone rotates the
    /// reference frame for every downstream bone. This is handled automatically
    /// because all constraints are evaluated against the parent's complete quaternion
    /// orientation, twist included.
    /// </summary>
    ITwistConstraint? TwistConstraint { get; set; }

    /// <summary>
    /// An optional name for this bone, used for identification and debugging.
    /// </summary>
    string? Name { get; set; }
}
