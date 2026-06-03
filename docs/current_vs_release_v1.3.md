# Current worktree vs release v1.3

## Scope

This comparison covers the changes from the existing `v1.3` tag to the current
`v1.4` release candidate.

## Main differences

- `v1.3` focused on guarded RTK outage attitude hold and loose 3D outage
  velocity-delta recovery.
- The current version adds the full RTK outage segmented Stage2 vertical
  recovery path:
  - gate-aware RTK vertical drift weighting;
  - outage-segmented drift/reference smoothing;
  - body-z and RTK-outage `ba_z` reestimate planning;
  - hard `ba_z` continuity breaks across reestimate boundaries;
  - causal pre-outage reference/fence support;
  - segmented batch execution for pre/outage/post sections;
  - standalone cutoff-equivalent pre-outage solving.

## Release behavior

The important release behavior is causal consistency before the first RTK outage:
post-outage information is no longer allowed to change the pre-outage vertical
solution through drift/reference, global Stage1 vertical initialization, or a
single cross-outage full-batch solve.

The promoted default configuration now tracks the validated segmented Stage2
run:

```text
runs/transformed1rtkjumpcut1_stage2_segmented_batch_standalone_pre_v2
```

The pre-outage portion of the full segmented result matches the standalone
cutoff run to numerical precision, with first dynamic 10 s slope matching at
approximately `-0.0914 mm/s`.
