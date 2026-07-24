# H2 r9 simple velocity diagnostic

This diagnostic is intentionally separate from the interactive Stage 06E
acceptance gate. It does not create or promote any stage gate.

Run the same bounded command through the official SDK client and the RoboDog
hardware abstraction:

```bash
bash scripts/09_pc2_h2_velocity_probe.sh vendor 0.20 0 0 1000
bash scripts/09_pc2_h2_velocity_probe.sh hal    0.20 0 0 1000
```

Arguments are:

```text
backend vx_mps vy_mps omega_radps duration_ms
```

The script automatically requires `MotionSwitcher form=0 name=ai`. Both
executables then require `FSM=601` before sending a non-zero command. Neither
path calls `Start`, `StandUp`, `Damp`, `Squat`, `Sit`, `SelectMode`, or
`ReleaseMode`.

The `vendor` backend sends one official
`LocoClient::SetVelocity(vx, vy, omega, duration)` RPC. The `hal` backend sends
the same velocity at 20 Hz through
`RobotHardwareInterface::writeRobotVelocityCommand()` for the requested
duration. Both paths call `StopMove()` at the end and print the vendor/project
return values.

Interpretation:

- vendor moves, HAL does not: inspect the RoboDog adapter or its watchdog.
- both move: the earlier r8/r9 Stage 06E profile or threshold was the issue.
- neither moves, with RPC return 0: the active H2 high-level controller is not
  consuming the velocity, or the selected velocity is below its effective
  motion threshold; inspect robot-side state/odometry before changing the HAL.

Use the protective stand, lock all stand casters, clear the area, and keep a
second operator on the original remote controller. Start with `0.20 0 0 1000`.
The official H2 example documents `0.5 0 0 1`, but do not increase to that
profile until the 0.20 m/s A/B result has been recorded.
