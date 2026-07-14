# Architecture

This project separates low-level robot control from policy inference.

## Layers
- `src/core/`: stable control data models and execution pipeline.
- `src/drivers/`: real Piper and mock robot backends.
- `src/policies/`: pluggable policy adapters.
- `scripts/`: runnable demos and bring-up helpers.

## Control flow
1. Driver reads `RobotState`.
2. Controller builds `Observation`.
3. Policy returns `Action`.
4. SafetyLayer validates and clamps.
5. Executor sends the safe action to the driver.

## ROS2 migration path
Future ROS2 integration should add bridge or node wrappers around the existing core APIs instead of rewriting them.
