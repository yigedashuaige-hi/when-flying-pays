# Phase F1 preregistration amendment 1: ideal-only diagnostic sentinels

**Time:** 2026-08-21, after the first technical launch and before inspecting
any Phase F1 success, energy, distance, switch, stuck, or slip outcome.

The original preregistration remains immutable at SHA-256
`5fe245aa81cdca4141ea53dcfc4aa1d875074a2d03e399a0732d9e8e6bb28841`.
Its all-column NaN/Inf stop rule was triggered by
`takeoff_clearance_available_m=Inf` in the first two ideal-backend CSVs. The
same schema also intentionally writes NaN for RotorS-only clearance,
reposition, and motor diagnostics when those systems are absent. These are
not missing Phase F1 outcomes: the frozen ideal backend defines those columns
as not applicable. Requiring finite RotorS-only diagnostics would make every
valid ideal run impossible without changing the frozen logger or backend.

The validity rule is therefore narrowed, before any substantive endpoint is
examined, as follows:

- every common task/state/perception/cost/outcome numeric field used by F1 must
  exist and be finite on every row;
- final `air_backend` must equal `ideal` and final `energy_model` must equal
  `legacy_task_distance_hover_proxy_v1`;
- backend-inapplicable RotorS, takeoff-clearance, handoff, contact, and
  reposition diagnostic fields may retain their existing NaN/Inf sentinel
  encoding and are excluded from F1 analysis;
- all original candidate sets, seeds, budgets, selection criteria, outcomes,
  tests, timeout settings, and stop rules for the required fields remain
  unchanged.

The two completed pre-amendment runs (seeds 100 and 101) and the interrupted
seed 102 launch are classified as technical-invalid attempts and retained in
`invalid_preflight_schema_sentinel/`. They are not silently replaced: the
exact reason and raw logs/CSVs remain available. The preregistered cells are
rerun from seed 100 under this schema-only amendment, equally for both
families. No candidate performance value from these attempts informed this
amendment.
