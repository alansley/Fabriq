namespace Fabriq;

/// <summary>
/// The result of a single FABRIK solve operation.
/// </summary>
public readonly struct SolveResult
{
    /// <summary>
    /// The final distance between the end effector and the target.
    /// </summary>
    public float Distance { get; init; }

    /// <summary>
    /// The number of iterations performed.
    /// </summary>
    public int Iterations { get; init; }

    /// <summary>
    /// Whether the end effector reached within <see cref="IFabrikChain.SolveDistanceThreshold"/>
    /// of the target.
    /// </summary>
    public bool Converged { get; init; }

    /// <summary>
    /// Whether the target was within the reachable range of the chain.
    /// A chain cannot reach a target further away than the sum of its bone lengths.
    /// </summary>
    public bool TargetReachable { get; init; }
}
