# Fabriq Constraint Model

## Overview

In Fabriq, a chain is a sequence of bones where each bone's valid orientations are
defined relative to its parent. The torso (or any fixed world-anchored point) is the
base, and every bone downstream is constrained relative to its predecessor:

```
base → bone₀ → bone₁ → bone₂ → ... → end effector
```

For example, a human arm might be modelled as:

```
torso → shoulder → upper arm → forearm → wrist → finger (x5)
```

Each bone carries its own constraint. The FABRIK solver enforces those constraints
after each pass, clamping any violated orientation back to the nearest valid one.
What those constraints *are* is entirely up to the user — Fabriq does not model
specific anatomy. You define the limits; the solver obeys them.

---

## Constraint Types

### 1. Unconstrained

No restriction. The bone may adopt any orientation regardless of its parent.
Useful for the end effector, or any joint that requires full rotational freedom
(a robot wrist that can spin freely, for example).

### 2. Rotor (Symmetric Cone)

The bone may deviate from its parent's reference direction by up to a specified
angle, equally in all directions. The valid region is a circular cone centred on
the parent bone's axis.

Carried over from Caliko. Good for ball-and-socket style joints where freedom
is roughly equal in all directions.

### 3. Hinge

Rotation is restricted to a single axis, with a minimum and maximum angle.
The bone cannot deviate laterally — only forward and backward along the hinge
plane.

Carried over from Caliko. Good for mechanical joints, and biological joints
such as the knee or the interphalangeal finger joints that are anatomically close
to a pure hinge.

### 4. Elliptical Cone (new in Fabriq)

The bone may deviate from its parent's reference direction by independently
configurable amounts in the horizontal (X) and vertical (Y) directions. The valid
region is an elliptical cone centred on the parent bone's axis.

Specified as `(x, y)` where each value is in the range `[0.0, 1.0]`:

| Value | Meaning |
|-------|---------|
| `0.0` | No freedom in this axis — locked to parent direction |
| `1.0` | Full 180° freedom in this axis — any orientation valid |

So `(1.0, 1.0)` is a full sphere: the bone may point in any direction including
directly back along its parent. `(0.0, 0.0)` is fully locked. Everything between
is yours to choose.

**There is no enforced maximum.** `(1.0, 1.0)` is mathematically valid and correct
for any use case that requires it. For reference, approximate values for some
common biological joints are given below — these are illustrative only, not
enforced by the library:

| Joint | Approximate (x, y) | Notes |
|-------|--------------------|-------|
| Human knuckle | `(0.05, 0.2)` | Narrow lateral, moderate flex |
| Human elbow | `(0.05, 0.45)` | Near-hinge; ~160° flex possible |
| Human wrist | `(0.15, 0.3)` | More flex than lateral deviation |
| Human shoulder | `(0.55, 0.65)` | Wide freedom, more vertical than lateral |
| Robot pivot joint | `(0.0, 0.5)` | Pure hinge, could also use Hinge type |
| Robot free joint | `(1.0, 1.0)` | No restriction |

A `(1.0, 1.0)` full-sphere joint is biologically implausible for most flesh-and-bone
use cases — a bone pointing directly back along its parent would imply self-intersection
in any realistic rig. But the library does not forbid it. Set sensible values for your
use case.

---

## Constraint Reference Frame

All constraints are evaluated relative to the **parent bone's current orientation**,
not world space. This means constraints travel with the chain — if you raise your
upper arm, the valid range of your forearm moves with it. The forearm's constraint
is always "X degrees from wherever the upper arm is pointing", not "X degrees from
world up".

This is the behaviour you want for any articulated chain of connected segments,
biological or mechanical.

---

## Twist Constraints

Twist — rotation of a bone around its own long axis — is a known future consideration.
A forearm can pronate and supinate (palm up to palm down); a robot tool head may need
to spin. This is not modelled in the current constraint system and is deferred to a
later version of Fabriq.

---

## 2D Usage

Fabriq operates in 3D space. For 2D use cases, set Z to zero and restrict rotations
to the XY plane. The constraint model applies identically — an elliptical cone with
zero Z component is effectively a 2D arc constraint. See the `Fabriq.Compat` wrappers
for convenience helpers that enforce this automatically.
