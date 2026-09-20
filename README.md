Making Brush Movement Visible
Dual-IMU Motion Analysis for Calligraphy Practice

Design research · Embodied interaction · Computational prototyping

How might motion sensing help learners reflect on the gestures behind a calligraphic stroke?

This project explores that question through motion sensing and computational analysis. Its design goal is to connect the visible ink trace with information about the gesture that produced it, supporting reflection during calligraphy practice.

This repository presents the implemented dual-IMU analysis pipeline. It transforms synchronized inertial measurements into three interpretable motion features and a configurable smoothness score, referred to here as MTSI. The complete implementation includes data preprocessing, windowed analysis, numerical tests, and reproducible demonstrations.

1. Design question

A finished ink trace records the outcome of a gesture, while a motion record offers a complementary view of its timing and variation. This project asks how that additional information could support reflection during calligraphy learning.

The proposed feedback makes selected aspects of movement inspectable while leaving room for human interpretation. Deliberate pauses, changes of direction, and variation in movement remain meaningful parts of writing; the motion record is intended to be read alongside the ink trace and a teacher's guidance.

2. Implementation highlights

- A common spatial reference: quaternion rotation places both sensors' acceleration readings in the same world coordinate system before gravity removal and projection onto a chosen test direction.
- Three inspectable features: rotation-rate variation, directional jerk, and acceleration residuals describe different aspects of the recorded signal.
- Data continuity handling: the pipeline detects sampling gaps, resets its analysis state, and resumes scoring after collecting a complete valid window.
- Reproducible evaluation: a runnable demonstration, eight automated tests, and CSV export make the processing steps and outputs accessible for review.

Both sensors contribute to a shared analysis window, with their individual feature values retained in the exported results.

3. From measurements to feedback

PROCESSING FLOW
Synchronized IMU pairs
  -> Acceleration: world-frame rotation -> gravity removal -> projection -> filtering
  -> Gyroscope: bias correction -> rotation-rate magnitude
  -> 100-sample analysis window
  -> Angular-change, jerk, and acceleration-residual scores
  -> Weighted MTSI

| Feature | Measurement | Interpretation |
| --- | --- | --- |
| Angular change | RMS of the time derivative of angular-velocity magnitude | Variation in rotation rate |
| Directional jerk | RMS of the time derivative of filtered directional acceleration | Rapid changes in acceleration along the selected direction |
| Acceleration residual | RMS deviation from each sensor's best-fit linear trend within the window | Local fluctuations beyond a linear acceleration trend |

Each measurement is mapped to a 0–100 component score:

S = 100 / (1 + (measurement / scale)²)
MTSI = 0.35 × S_omega + 0.40 × S_jerk + 0.25 × S_a


The default scales are 3.0 rad/s², 12.0 m/s³, and 0.6 m/s² respectively. The scales and weights are configurable, allowing their effects on the score to be examined in subsequent experiments.

At 100 Hz, the method uses a 100-sample window and an 8 Hz acceleration filter. An interval greater than 15 ms resets the filter and window. Scores become available after 101 uninterrupted samples at startup because the first sample has no preceding value for differentiation.

4. Evaluation

The implementation was evaluated using reproducible synthetic inputs and analytical reference cases. The demonstration contains three four-second conditions:

| Condition | Purpose |
| --- | --- |
| Stationary | Establish a stable baseline and check initialization behavior |
| Smooth translation | Examine the response to gradual acceleration changes |
| Abrupt periodic translation | Examine the response to stronger, faster acceleration changes |

The demonstration keeps orientation constant and angular velocity zero. It exercises the acceleration and jerk pathways; angular-change scoring is tested separately using an analytical reference case.

All eight automated tests passed in the local verification run. Checks cover coordinate rotation, gravity removal, bias correction, startup behavior, gap recovery, contributions from both sensors, invalid inputs, and known score values. In the analytical reference case, component scores of 50, 50, and 100 produce an MTSI of 62.5. File replay also reproduced the generated demonstration output exactly.

MTSI is interpreted here as a preliminary signal-smoothness indicator, alongside its three component scores and the movement context. Stillness can also score 100; the score describes selected signal properties rather than overall handwriting quality.

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

Integration

The module accepts synchronized sensor pairs: timestamps in seconds; acceleration in m/s² including gravity; gyroscope readings in rad/s; and wxyz body-to-world quaternions in a shared +Z-up world frame. It applies supplied per-sensor gyroscope biases and projects acceleration onto a selected world-frame direction, defaulting to +X. Use zero bias values for inputs already corrected upstream.

The input interface connects to an upstream acquisition system that supplies synchronized readings and orientation estimates. Input examples and integration instructions are provided in the Chinese guide.

6. Future development

Further development will focus on connecting this computational foundation to situated calligraphy practice:

1. Hardware evaluation: compare synchronized sensor recordings with video annotations and examine the effects of sensor arrangement and mounting position.
2. Learning interaction: explore how learners and teachers interpret the component signals and use them to reflect on intentional changes, pauses, and stroke transitions.
3. Multimodal feedback: investigate the alignment of motion records with camera-derived ink regions and stroke-shape features.
