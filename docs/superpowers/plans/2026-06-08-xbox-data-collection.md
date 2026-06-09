# XBOX Data Collection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a first-class XBOX data collection path that preserves the existing PICO collection path and writes LeRobot-compatible datasets with a dedicated `observation.gamepad` feature.

**Architecture:** Keep `record_sdk_episode.py` for PICO. Add a ROS2-side XBOX state exporter that writes robot and gamepad snapshots to `/tmp`, and add `record_xbox_episode.py` that records cameras, robot state, XBOX input, and LeRobot data. Extend the writer so controller input features are selected by collection mode.

**Tech Stack:** Python, ROS2 `rclpy`, `sensor_msgs/Joy`, `sensor_msgs/JointState`, existing `teleop_data_collection` writer, pytest.

---

### Task 1: Controller Input Schema

**Files:**
- Modify: `teleop_data_collection/lib/lerobot_writer.py`
- Test: `teleop_data_collection/tests/test_lerobot_compat.py`

- [x] Add tests that build PICO and gamepad feature schemas separately.
- [x] Implement a `controller_observation` option with `pico`, `gamepad`, and `none`.
- [x] Keep PICO schema unchanged for existing datasets.

### Task 2: XBOX Input Reader

**Files:**
- Create: `teleop_data_collection/lib/gamepad.py`
- Test: `teleop_data_collection/tests/test_lerobot_compat.py`

- [x] Add tests for stale/missing gamepad JSON and normalized vector flattening.
- [x] Implement `read_gamepad_state()` and `flatten_gamepad_observation()`.

### Task 3: ROS2 XBOX State Exporter

**Files:**
- Create: `teleop_data_collection/scripts/export_ros_xbox_state.py`

- [x] Subscribe to `/joint_states` and `/joy`.
- [x] Export `/tmp/robot_latest_state.json` and `/tmp/xbox_latest_input.json`.
- [x] Use Start long-press as `episode_done=true`.

### Task 4: XBOX Episode Recorder

**Files:**
- Create: `teleop_data_collection/scripts/record_xbox_episode.py`
- Modify: `teleop_data_collection/lib/episode.py`

- [x] Reuse the camera capture, preview, and stop behavior from `record_sdk_episode.py`.
- [x] Write `controller_inputs.jsonl` raw logs.
- [x] Write `observation.gamepad` in LeRobot frames.

### Task 5: Documentation And Verification

**Files:**
- Modify: `teleop_data_collection/README.md`

- [x] Document the XBOX collection terminals and button semantics.
- [x] Run focused pytest for LeRobot compatibility.
