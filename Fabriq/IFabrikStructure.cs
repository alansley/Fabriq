using System.Numerics;

namespace Fabriq;

/// <summary>
/// A structure is the top-level container in Fabriq's hierarchy:
///
///   IFabrikStructure → IFabrikChain → IFabrikBone
///
/// A structure owns one or more chains and orchestrates their solve. The classic
/// example is a humanoid body: the torso is the structure, and each limb (left arm,
/// right arm, left leg, right leg, spine, head) is a chain within it. A hand is
/// another natural structure: the palm is the base, and each finger is a chain.
///
/// ╔═══════════════════════════════════════════════════════════════════════════╗
/// ║  FUNDAMENTAL: THE SOLVE IS SEQUENTIAL AND OUTWARD FROM THE ROOT         ║
/// ║                                                                          ║
/// ║  A structure is NOT solved in parallel. Chains are solved one at a time, ║
/// ║  starting from the root chain and working outward through the dependency ║
/// ║  tree. Each chain's BasePosition is updated from its parent's solved     ║
/// ║  joint position BEFORE that chain is solved.                             ║
/// ║                                                                          ║
/// ║  This means a child chain's solve is always based on where its parent   ║
/// ║  actually ended up — not where it started. Position flows outward from  ║
/// ║  the structure root, exactly as it does in a real articulated body.     ║
/// ║                                                                          ║
/// ║  Example — a humanoid arm:                                               ║
/// ║    1. Solve spine  → spine result is now authoritative                  ║
/// ║    2. Update shoulder base from spine's solved joint position            ║
/// ║    3. Solve upper arm → upper arm result is now authoritative            ║
/// ║    4. Update forearm base from upper arm's solved end effector           ║
/// ║    5. Solve forearm → and so on to wrist, then each finger              ║
/// ║                                                                          ║
/// ║  Attempting to solve chains in parallel or in arbitrary order will       ║
/// ║  produce incorrect results. The structure enforces solve order.          ║
/// ╚═══════════════════════════════════════════════════════════════════════════╝
///
/// TARGET RESOLUTION
/// -----------------
/// A structure has a single default target. Each chain within the structure may
/// optionally declare its own target, which overrides the structure's target for
/// that chain only. Chains without their own target inherit the structure's target.
///
/// This design supports two common use cases cleanly:
///
///   1. Single target: set the structure target, all chains solve toward it.
///      Example: a tentacle cluster all reaching for the same point.
///
///   2. Multiple targets: set a target per chain. The structure target acts as a
///      fallback for any chain that does not declare its own.
///      Example: a hand where each finger tracks a different contact point.
///
/// CHAIN CONNECTIONS AND ANCHOR POINTS
/// -------------------------------------
/// The structure is a container — it does not define how chains connect to each
/// other. That is each chain's own responsibility. A chain declares its anchor
/// via ParentChain and ParentJointIndex: the joint on another chain that its base
/// tracks. If ParentChain is null, the chain is anchored to world space via its
/// BasePosition.
///
/// This means constraints at a junction belong to the connecting chain, not to
/// the joint being connected to. A finger brings its own knuckle constraints to
/// the hand — the hand does not impose constraints on the finger. This keeps
/// constraint ownership unambiguous and makes chains self-contained units that
/// can be moved between structures without losing their behaviour.
///
/// SOLVE RESULTS
/// -------------
/// Solve() returns one SolveResult per chain, in root-outward dependency order.
/// The results are sequential — result[i] reflects the state of chain[i] after
/// its parent chains have already been solved and its BasePosition updated.
///
/// No aggregate result is provided. An aggregate (e.g. average distance across
/// all end effectors) would be meaningless — "how close is the hand to the target"
/// is a question about the structure's base position, not about the IK solve.
/// Per-chain results give the caller actionable diagnostic data: which chains
/// converged, which did not, and by how much. A caller that wants a summary can
/// compute it from the list.
/// </summary>
public interface IFabrikStructure
{
    /// <summary>
    /// The chains within this structure, in dependency order — parent chains
    /// appear before any chain that is anchored to them. The structure enforces
    /// this ordering during solve.
    /// </summary>
    IReadOnlyList<IFabrikChain> Chains { get; }

    /// <summary>
    /// The default target for all chains in this structure. Any chain that does
    /// not declare its own target will solve toward this position.
    /// </summary>
    Vector3 Target { get; set; }

    /// <summary>
    /// The world-space position of the structure's root — typically the base of
    /// the first chain (e.g. the torso, the palm, the shoulder girdle). This is
    /// the fixed anchor from which all chain positions are derived.
    /// </summary>
    Vector3 BasePosition { get; set; }

    /// <summary>
    /// An optional name for this structure, used for identification and debugging.
    /// </summary>
    string? Name { get; set; }

    /// <summary>
    /// Solves all chains in this structure toward their respective targets.
    ///
    /// Before solving each chain, the structure updates that chain's BasePosition
    /// from its parent chain's joint position (if the chain has a parent). Chains
    /// are solved in dependency order so that parent chain positions are always
    /// current before a child chain is solved.
    ///
    /// Each chain uses its own target if one is set, otherwise it falls back to
    /// the structure's <see cref="Target"/>.
    ///
    /// Returns one <see cref="SolveResult"/> per chain, in chain order. See the
    /// class summary for why no aggregate result is provided.
    /// </summary>
    IReadOnlyList<SolveResult> Solve();

    /// <summary>
    /// Adds a chain to the structure. If the chain has a ParentChain, it will be
    /// inserted after its parent in the solve order. If it has no parent, it is
    /// added as a root chain anchored to world space.
    /// </summary>
    void AddChain(IFabrikChain chain);

    /// <summary>
    /// Removes a chain from the structure. Any chains that are anchored to the
    /// removed chain will have their ParentChain set to null, making them
    /// world-anchored at their current BasePosition.
    /// </summary>
    void RemoveChain(IFabrikChain chain);
}
