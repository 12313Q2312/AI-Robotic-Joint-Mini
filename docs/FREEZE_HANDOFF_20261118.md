# Maintenance freeze and safety handoff (2026-11-18)

## Scope and evidence
This repository is paused after the RJ-INT-01 integration and sensor non-finite-value handling work. CI and headless Webots smoke tests are **simulation evidence only**, not physical safety certification. Refer to [RJ-INT-01](RJ-INT-01.md), [limitations](LIMITATIONS.md), and the repository's GitHub Actions for the exact tested revision.

## Accepted at freeze
- C++ actuator torque boundary and parser regression tests are part of CI.
- ROS 2 build and normal-path headless Webots smoke run in CI.
- Non-finite feedback handling has unit-level coverage.

## Not accepted / do not claim
- No physical actuator, wearable, or human-subject validation.
- No Webots plugin-level NaN/Inf injection acceptance for the fault path.
- Zero commanded torque is **not** a safe holding or braking strategy under gravity.
- No demonstrated real-time scheduling or mechanical fail-safe behavior.

## Reopening gate
1. Reproduce the current default-branch CI with exact commit and logs.
2. Inject NaN, +Inf and -Inf in the actual Webots plugin path; verify same-step zero torque, independent fault topic, latched fault and recovery only by intended reset.
3. Verify message-array lengths, timing, 24 N·m limit and failure behavior.
4. Before hardware or human testing: separately review mechanical restraints, braking, power loss, emergency stop, load limits and risk assessment.
5. Record test environment, raw logs, pass/fail and reviewer sign-off in a new PR.

No physical safety claims or hardware deployment are authorized by this freeze document.
