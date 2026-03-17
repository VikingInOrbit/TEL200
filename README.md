# TEL200 YuMi Lab - Left Arm RAPID Documentation (Updated)

## Overview
This setup uses only the left arm (`T_ROB_L`). The active program now supports two run modes:

- **Toggle mode**: object buttons move objects left ↔ right.
- **Stack mode**: object buttons stack one of each object.

The right-arm task stays parked and performs no motion.

## Active File

Only update `activefileleft` in the workspace root.

## Digital Inputs

| RAPID signal | Alias name | Purpose |
|--------------|------------|---------|
| di_cube | custom_DI_0 | Cube action (toggle/stack by mode) |
| di_cylinder | custom_DI_1 | Cylinder action (toggle/stack by mode) |
| di_prism | custom_DI_2 | Prism action (toggle/stack by mode) |
| di_EmergencySituation | custom_DI_3 | Emergency stop, active low |
| di_home | custom_DI_4 | Short press = mode action, hold 5 s = mode switch |

Alias binding is done with string names and `AliasIO`.

## Mode Behavior

### Toggle mode (default)
- Press object button once: move left → right.
- Press again: move right → left.

### Stack mode
- Press object button: stack that object (max one of each in stack).
- Short press `di_home`: unstack all in reverse order.

### Mode switching
- Hold `di_home` for 5 seconds to switch mode.
- On switch, robot moves to dedicated `HomeL_mode_switch`, then returns home.

## State Tracking

Object state is array-based:

- `obj_is_right{}`: object currently on right side.
- `obj_is_stacked{}`: object currently in stack.
- `stack_order{}`: push order in stack.
- `stack_was_right{}`: remembers if each stacked object came from right side.

This ensures unstacking returns each object to the **same side it came from**.

## Parameterization

The code is simplified using shared constants for tuning:

- **Motion**: `spd_fast`, `spd_pick`, `spd_stack`, `zone_path`
- **Timing**: `wait_grip_in`, `wait_grip_out`, `loop_dt`, `mode_hold_sec`
- **Geometry**: `x_cube`, `x_cylinder`, `x_prism`, `y_left`, `y_right`, pickup/approach heights
- **Stacking**: `z_stack_base`, `stack_step_z`

Stack heights are generated from a base target using `Offs(...)`.

## Motion Structure

### Shared motion primitives
- `MoveObjectSideToSide(...)` handles generic pick-and-place flow.
- `MoveObjectLeftToRight(object_id)` and `MoveObjectRightToLeft(object_id)` select object paths.
- `StackFromSide(...)` and `UnstackToSide(...)` handle stack transfer.

### Home behavior
- `GoHome` uses `MoveJ` via `HomeL_travel` to `HomeL`.
- Main motion procedures use `HomeL_travel` as transit point to avoid unnecessary home jitter.

## Emergency Handling

- Emergency input (`di_EmergencySituation=0`) triggers trap.
- Motion stops with `StopMove`, resumes after signal returns high.

## Right Arm Status

Right arm remains disabled in runtime behavior (parked/idle logic only).

## Notes

- `ConfJ\Off` and `ConfL\Off` are enabled before home/motion procedures.
- Prism retains its specific orientation (`[0,0.707107,0.707107,0]`) for reliable handling.
- If RobotStudio regenerates module filenames, reapply this logic to the active left-arm file.
