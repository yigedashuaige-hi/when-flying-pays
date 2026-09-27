# Phase E1a-R2 Persistent Safe-Site Design

Date: 2026-08-28 (Asia/Shanghai)

## Scope and frozen boundary

This phase closes one RotorS execution/recovery interface: a valid launch site
must remain available as a navigation candidate across an AIR/LAND cycle. It
does not change Scene A, its route or fixed-switch request sequence, policy
logic, Lee gains, vehicle dynamics, takeoff safety volume, clearance rules,
mission timeout, success criteria, D2, Stage 0, or F1. The ideal backend is not
part of the new experiment. The historical E1a-R `success=0` artifacts remain
immutable.

## Root cause established from frozen R1 evidence

The R1 coordinator stores valid ground samples in a 12-s deque. Samples are
pruned by simulation time and are intended to represent a recently traversed
reverse ground path. After the first flight and landing, the second TAKEOFF
request at 30.020 s has neither a recent reverse-path sample nor any separate
cross-sortie site representation, so it returns
`takeoff_reposition_failed:no_stable_valid_history`. The vehicle remains
ground-owned and disarmed. The clearance sensor and the first
GROUND--TAKEOFF--AIR--LAND--DISARM--GROUND handshake worked as designed.

## Considered approaches

1. **Increase the 12-s history window.** Rejected. This would make an old pose
   look like a recently reversible ground path and would conflate historical
   clearance evidence with current reachability.
2. **Add a persistent safe-site registry plus bounded ground verification.**
   Selected. The registry retains measured valid sites across sorties, but an
   old observation is only a navigation hint. It can never authorize takeoff
   without current, fresh, stable clearance at the reached pose.
3. **Add a general local/global planner.** Rejected for R2. It is a larger
   subsystem and exceeds the requested minimal interface repair.

## Data model

The existing recent reverse-path deque remains unchanged in meaning. A second,
bounded registry stores deduplicated safe-site records:

- generated site identifier;
- measured world-frame x, y, yaw;
- observation and last-validation simulation timestamps;
- `measurement_valid`, `site_valid`, and stable-valid evidence;
- measured available/required vertical clearance, nearest-obstacle distance,
  and geometry-derived recheck distance;
- source (`stable_ground` or `approved_takeoff`);
- ground epoch and validation count;
- reachability state for the current recovery episode.

Records enter the registry only while public mode is GROUND, ground ownership
is true, motors are disabled, the atomic site measurement is fresh, and the
existing stable-valid hold has completed. The approved takeoff pose is also
upserted from the same fresh stable-valid measurement immediately before
ground ownership is released. No scene coordinate or collision name is an
input.

## Recovery and authorization contract

Candidate priority is:

1. the existing recent reverse-path target, whose reachability evidence is the
   just-traversed ground path;
2. a persistent site, ordered deterministically by current XY distance and
   excluding any site already shown unreachable in the current episode.

A persistent site's old clearance observation selects only a ground goal. The
existing ground controller remains the sole `/mecanum/cmd_vel` publisher. The
coordinator publishes only a pose override and active flag. During reposition,
public mode remains GROUND, ground ownership remains true, and motor enable is
false.

Arrival requires the existing 0.03-m XY tolerance and 0.05-m/s measured speed
limit plus a *current* fresh stable-valid atomic clearance measurement. Thus a
stale or invalid stored observation cannot authorize flight. Successful
arrival marks the site reachable in the current ground epoch and proceeds
through the unchanged brake, release, arm, vertical takeoff, target-height
clearance, and horizontal-release gates.

If a persistent target makes no progress or reaches the existing bounded
reposition timeout, it is marked unreachable for that recovery episode and is
not immediately retried. With no usable candidate, the existing fail-safe
GROUND behavior remains active: the recovery override is removed, the nominal
ground controller can continue its route, and the coordinator may leave the
failure state only if the *current* footprint becomes freshly stable-valid
while the upstream TAKEOFF request is still present. This is a generic
deferred-takeoff recovery, not a bypass: takeoff still traverses every existing
handoff and clearance gate.

## Tests and stop rule

Before the frozen Scene A run:

- host-side unit tests cover cross-sortie persistence, deterministic
  deduplication, stale evidence being unable to authorize takeoff, invalid
  records being rejected, and per-episode unreachable exclusion;
- isolated Gazebo tests cover one cross-sortie persistent-site recovery,
  bounded unreachable behavior, motor-zero reposition, no simultaneous ground
  and rotor ownership, and sampled handoff/clearance/contact regressions.

Only if those checks pass is the one frozen Scene A RotorS case run. Any NaN,
ownership overlap, invalid-site arm, set-model-state client, residual process,
or genuine mission failure stops R2. Parameters are not iteratively changed to
force DONE.

## Paper gate

The manuscript remains unchanged unless the exact run reaches `DONE` with
`success=1` and `timeout=0`. A passing result may update only the RotorS
engineering-validation claims and derived timeline figure. Stage 0/F1 policy
claims, numerical results, estimands, and energy semantics remain frozen.
