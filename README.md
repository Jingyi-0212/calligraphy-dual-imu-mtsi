Making Brush Movement Visible
Dual-IMU Motion Analysis for Calligraphy Practice

Design research · Embodied interaction · Computational prototyping

How might motion sensing help learners reflect on the gestures behind a calligraphic stroke?

This project explores that question through a computational prototype for a calligraphy-support system. The intended interaction connects the visible ink trace with information about how the brush moves, giving learners another way to examine their practice alongside a teacher's guidance.

This repository contains the dual-IMU signal-processing prototype: a runnable Python implementation that converts synchronized inertial measurements into three interpretable motion features and a preliminary smoothness score, referred to here as MTSI.

Current stage: implemented and tested with synthetic data and analytical reference cases. The examples in this repository are simulated; physical-device and learner-study validation remain future work for this standalone implementation.

1. Design question

A finished ink trace records the outcome of a gesture, while a motion record offers a complementary view of its timing and variation. This project asks how that additional information could support reflection during calligraphy learning.

The design goal is to make selected aspects of movement inspectable. A smoothness score alone cannot establish the quality or expressiveness of a stroke: deliberate pauses, changes of direction, and variation in movement may be meaningful parts of writing.

2. What this prototype demonstrates

- A common spatial reference: quaternion rotation places both sensors' acceleration readings in the same world coordinate system before gravity removal and projection onto a chosen test direction.
- Three inspectable features: rotation-rate variation, directional jerk, and acceleration residuals describe different aspects of the recorded signal.
- Explicit handling of unavailable data: initialization and sampling gaps produce a waiting period rather than a misleading score.
- Reproducible evaluation: a synthetic demonstration and eight numerical tests allow a reviewer to inspect and run the method without sensor hardware.

The two sensors contribute to the same analysis window. Whether this arrangement offers an advantage over one sensor is an open experimental question, not a demonstrated result of this repository.

3. From measurements to feedback

PROCESSING FLOW
Synchronized IMU pairs
  -> Acceleration: world-frame rotation -> gravity removal -> projection -> filtering
  -> Gyroscope: bias correction -> rotation-rate magnitude
  -> 100-sample analysis window
  -> Angular-change, jerk, and acceleration-residual scores
  -> Weighted preliminary MTSI

| Feature | Measurement | Interpretation |
| --- | --- | --- |
| Angular change | RMS of the time derivative of angular-velocity magnitude | Variation in rotation rate |
| Directional jerk | RMS of the time derivative of filtered directional acceleration | Rapid changes in acceleration along the selected direction |
| Acceleration residual | RMS deviation from each sensor's best-fit linear trend within the window | Local fluctuations beyond a linear acceleration trend |

Each measurement is mapped to a 0–100 component score:

S = 100 / (1 + (measurement / scale)²)
MTSI = 0.35 × S_omega + 0.40 × S_jerk + 0.25 × S_a


The default scales are 3.0 rad/s², 12.0 m/s³, and 0.6 m/s² respectively. These scales and weights are configurable prototype parameters; they have not been fitted to expert assessments of calligraphy.

At 100 Hz, the method uses a 100-sample window and an 8 Hz acceleration filter. An interval greater than 15 ms resets the filter and window. Scores become available after 101 uninterrupted samples at startup because the first sample has no preceding value for differentiation.

4. Evidence and interpretation

The supplied demonstration contains three four-second conditions:

| Synthetic condition | Purpose |
| --- | --- |
| Stationary | Establish initialization behavior and show that stillness can score 100 |
| Smooth translation | Examine the response to gradual acceleration changes |
| Abrupt periodic translation | Examine the response to stronger, faster acceleration changes |

The demonstration keeps orientation constant and angular velocity zero. It exercises the acceleration and jerk pathways; angular-change scoring is tested separately using an analytical reference case.

All eight automated tests passed in the local verification run. Checks cover coordinate rotation, gravity removal, bias correction, startup behavior, gap recovery, contributions from both sensors, invalid inputs, and known score values. In the analytical reference case, component scores of 50, 50, and 100 produce an MTSI of 62.5. File replay also reproduced the generated demonstration output exactly.

These results establish reproducible implementation behavior. They do not demonstrate improved learning outcomes or a validated distinction between skilled and unskilled writing.

5. Run and inspect

Requires Python 3.9+ and the standard library only. Download the repository using Code → Download ZIP, extract it, and run these commands in its folder:

python dual_imu_mtsi.py --demo --output demo_results.csv
python -m unittest -v test_dual_imu_mtsi.py


The demonstration produces 1,200 samples and 1,100 valid scores after initialization. Empty initial score cells mean that the analysis window is not yet ready.

For file replay:

python dual_imu_mtsi.py --input demo_input.jsonl --output replay_results.csv


| File | What to inspect |
| --- | --- |
| dual_imu_mtsi.py | Complete preprocessing, scoring, and demonstration code |
| test_dual_imu_mtsi.py | Numerical reference cases and boundary-condition checks |
| demo_input.jsonl | Synthetic synchronized sensor pairs |
| demo_results.csv | Generated features and scores |
| 使用说明.md | Chinese usage guide and integration examples |

Sensor input assumptions

Inputs must already be synchronized and calibrated: timestamps in seconds; acceleration in m/s² including gravity; gyroscope readings in rad/s; and wxyz body-to-world quaternions in a shared +Z-up world frame. The default projection direction is world +X. The program accepts per-sensor gyroscope biases; zero defaults do not imply that calibration has been performed.

Serial acquisition, synchronization, and orientation estimation belong to the upstream system and are outside this standalone module. Input examples and integration instructions are provided in the Chinese guide.

6. Research directions

The next stage would examine whether the features support useful reflection in real calligraphy practice:

1. Validate the measurements: compare synchronized sensor recordings with video annotations, including pauses, intentional changes, and missing-data conditions.
2. Evaluate the sensor arrangement: compare one- and two-sensor configurations and assess sensitivity to mounting position and orientation error.
3. Study interpretation with learners and teachers: investigate which feedback is understandable and useful, and when a single score hides meaningful variation.
4. Explore multimodal feedback: align motion records with camera-derived ink regions and stroke-shape features, keeping these distinct from measured brush-tip trajectories.

Current limitations guide these questions: stillness can receive a high score; rotation-axis changes at constant angular-speed magnitude may be missed; and acceleration is evaluated along a fixed experimental direction. This code does not estimate brush-tip position, linear speed, ink shape, or handwriting quality.

The repository provides an inspectable foundation for investigating how computational feedback might support an embodied craft while retaining the role of human interpretation.
