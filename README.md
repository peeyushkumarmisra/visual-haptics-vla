# visual-haptics-vla

**Can a robot "feel" an object's stiffness and mass just by watching itself squeeze and lift it?**

A physics-informed, RT-1-style Vision-Language-Action (VLA) model for a 4-DOF OpenManipulator-X in ROS 2 Jazzy + Gazebo Harmonic. The arm probes a "mystery block", infers stiffness and mass from vision alone (no force sensing), then sorts it into the bin named in a language instruction. Designed to train and run on a 4 GB GPU.

> 🚧 Work in progress. Phase 2 (data pipeline) is complete; model training is next.

## Planned architecture
- FiLM-conditioned MobileNetV3 + TokenLearner + causal transformer, trained by imitation learning
- Optical-strain physics loss, plus a runtime anti-crush / anti-slip guardrail
- FP16/INT8 ONNX export for a 4 GB GPU

## What is built so far
- **Simulation arena** (`rt1_sim_bringup`): sorting world, arm with dual cameras, ros2_control, Gazebo bridge; finger-damping sweep for stable contact (`data/g2_finger_damping_sweep.csv`)
- **Gripper controller** (`rt1_gripper_controller`): custom C++ `ros2_control` gripper-centering controller
- **Vision measurement** (`rt1_data_pipeline`, C++): dual-camera stitcher; probe metrics via HSV segmentation → connected components → sub-pixel centroids (fingertip gap, visual strain)
- **Data pipeline**: mystery-block spawner with hidden (k, m) — log-uniform stiffness, disjoint train/eval ranges, balanced prompt × class design, seeded schedules — plus episode logger, dataset audit and replay tools
- **Custom interfaces** (`rt1_interfaces`): FSM status, probe metrics, episode info

## Progress gates
| Gate | Status |
|---|---|
| G2 – contact physics tuned (finger damping sweep) | ✅ |
| G3 – dataset collected and verified (train 299/304, eval 64/64, replay 3/3) | ✅ |
| Model training and ablations | ⏳ |

## Run (Docker, WSL2)
```bash
cd docker && docker compose up -d
docker compose exec sim bash
colcon build --symlink-install && source install/setup.bash
ros2 launch rt1_sim_bringup sim_arena.launch.py
```

**Stack:** C++17 · Python · ROS 2 Jazzy · Gazebo Harmonic · ros2_control · OpenCV · PyTorch · ONNX Runtime · Docker
