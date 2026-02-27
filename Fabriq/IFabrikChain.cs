using System.Numerics;

namespace Fabriq;

/// <summary>
/// An ordered chain of bones solved by the FABRIK algorithm.
///
/// The chain owns the world-space context that individual bones lack. Bones store
/// orientation and length only; the chain provides the base position from which
/// all joint positions are derived by traversing bone orientations and lengths.
///
/// Joint indexing: joint 0 is the base (p₁ in the paper), joints 1..Bones.Count
/// are the distal end of each bone in order. The end effector is joint Bones.Count.
/// There are therefore Bones.Count + 1 joints in total.
///
/// Internally, the solver works with joint positions as described in the original
/// FABRIK paper (Aristidou and Lasenby, 2011). The quaternion orientations stored
/// on each bone are derived from those positions after each solve.
///
/// PARENT CHAIN CONNECTION
/// -----------------------
/// A chain may be anchored to a specific joint on another chain via
/// <see cref="ParentChain"/> and <see cref="ParentJointIndex"/>. When set, the
/// structure updates this chain's BasePosition from that joint before solving.
/// Constraints at the junction belong to this chain — the parent chain does not
/// impose constraints on its children. If ParentChain is null, the chain is
/// anchored to world space via BasePosition directly.
///
/// PER-CHAIN TARGET
/// ----------------
/// A chain may declare its own target via <see cref="Target"/>, overriding the
/// structure's default target. When null, the chain inherits the structure target.
/// This allows a hand structure to solve each finger toward a different contact
/// point while sharing a single structure-level fallback.
/// </summary>
public interface IFabrikChain
{
    /// <summary>
    /// The bones in this chain, ordered from base to end effector.
    /// </summary>
    IReadOnlyList<IFabrikBone> Bones { get; }

    /// <summary>
    /// The world-space position of the chain's base joint (p₁ in the paper).
    /// </summary>
    Vector3 BasePosition { get; set; }

    /// <summary>
    /// Whether the base joint is fixed in world space. When true, the backward
    /// pass restores the base to its original position after each iteration.
    /// When false, the base may translate — used for floating chains or chains
    /// connected to a sub-base in a structure.
    /// </summary>
    bool FixedBase { get; set; }

    /// <summary>
    /// The total length of the chain — the sum of all bone lengths. Used to
    /// determine whether a target is reachable before iterating.
    /// </summary>
    float ChainLength { get; }

    /// <summary>
    /// Maximum number of iterations before the solver gives up.
    /// </summary>
    int MaxIterations { get; set; }

    /// <summary>
    /// The solver stops when the end effector is within this distance of the target.
    /// </summary>
    float SolveDistanceThreshold { get; set; }

    /// <summary>
    /// The solver stops when the change in end effector position between iterations
    /// falls below this value — handles cases where the chain is oscillating near
    /// but not reaching the target.
    /// </summary>
    float MinIterationChange { get; set; }

    /// <summary>
    /// The chain this chain's base is anchored to, if any. When set, the structure
    /// will update this chain's BasePosition from the joint at ParentJointIndex on
    /// the parent chain before each solve. When null, this chain is world-anchored.
    /// </summary>
    IFabrikChain? ParentChain { get; set; }

    /// <summary>
    /// The index of the joint on <see cref="ParentChain"/> that this chain's base
    /// tracks. Ignored when ParentChain is null. Joint 0 is the parent's base;
    /// joint ParentChain.Bones.Count is the parent's end effector.
    /// </summary>
    int ParentJointIndex { get; set; }

    /// <summary>
    /// An optional per-chain target that overrides the structure's default target.
    /// When null, this chain inherits the structure's target during solve.
    /// </summary>
    Vector3? Target { get; set; }

    /// <summary>
    /// An optional name for this chain, used for identification and debugging.
    /// </summary>
    string? Name { get; set; }

    /// <summary>
    /// Solve the chain for the given target position. Runs forward and backward
    /// FABRIK passes iteratively until the end effector converges on the target,
    /// the change between iterations drops below <see cref="MinIterationChange"/>,
    /// or <see cref="MaxIterations"/> is reached.
    /// </summary>
    /// <param name="target">The world-space position to solve toward.</param>
    /// <returns>A <see cref="SolveResult"/> describing the outcome.</returns>
    SolveResult Solve(Vector3 target);

    /// <summary>
    /// Returns the world-space position of joint <paramref name="index"/>.
    /// Joint 0 is the base; joint Bones.Count is the end effector.
    /// </summary>
    Vector3 GetJointPosition(int index);

    /// <summary>
    /// Adds a bone to the end of the chain.
    /// </summary>
    void AddBone(IFabrikBone bone);

    /// <summary>
    /// Removes the bone at the given index from the chain.
    /// </summary>
    void RemoveBone(int index);
}
