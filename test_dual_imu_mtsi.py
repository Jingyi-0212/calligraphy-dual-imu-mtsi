"""Run: python -m unittest -v test_dual_imu_mtsi.py"""
import math
import unittest

from dual_imu_mtsi import (GRAVITY, DualIMUPipeline, IMUPreprocessor,
                           MTSIAnalyzer, demo_records, rotate)


class SignalTests(unittest.TestCase):
    def stationary(self, pipeline, t):
        return pipeline.update(t, [GRAVITY]*2, [(0, 0, 0)]*2, [(1, 0, 0, 0)]*2)

    def test_quaternion_rotation(self):
        result = rotate((math.sqrt(.5), 0, 0, math.sqrt(.5)), (1, 0, 0))
        for actual, expected in zip(result, (0, 1, 0)):
            self.assertAlmostEqual(actual, expected)

    def test_tilted_stationary_sensor_and_bias(self):
        q = (math.sqrt(.5), math.sqrt(.5), 0, 0)
        bias = (.01, -.02, .03)
        p = IMUPreprocessor(direction_unit=(0, 0, 1), gyro_biases=[bias]*2)
        sample = p.update(0, [(0, GRAVITY[2], 0)]*2, [bias]*2, [q]*2)
        for a in sample['acceleration']:
            self.assertAlmostEqual(a, 0)
        self.assertEqual(sample['omega'], [0, 0])

    def test_startup_and_stationary_score(self):
        pipeline = DualIMUPipeline()
        for k in range(100):
            _, score = self.stationary(pipeline, k/100)
            self.assertIsNone(score)
        _, score = self.stationary(pipeline, 1.0)
        self.assertEqual(score['MTSI'], 100)

    def test_gap_restarts_filter_and_warmup(self):
        pipeline = DualIMUPipeline()
        for k in range(101):
            self.stationary(pipeline, k/100)
        sample, score = pipeline.update(2, [(3, 0, GRAVITY[2])]*2,
                                        [(0, 0, 0)]*2, [(1, 0, 0, 0)]*2)
        self.assertTrue(all(math.isnan(v) for v in sample['jerk']))
        self.assertIsNone(score)
        for k in range(1, 100):
            _, score = self.stationary(pipeline, 2+k/100)
            self.assertIsNone(score)
        _, score = self.stationary(pipeline, 3)
        self.assertIsNotNone(score)

    def test_known_analytic_scores(self):
        # |omega|=3t -> angular RMS=3; a=12t -> jerk RMS=12;
        # a is exactly linear -> its detrended residual=0.
        analyzer = MTSIAnalyzer()
        for k in range(100):
            t = k/100
            score = analyzer.update(dict(timestamp=t, omega=[3*t]*2,
                                         acceleration=[12*t]*2, jerk=[12]*2))
        self.assertAlmostEqual(score['S_omega'], 50)
        self.assertAlmostEqual(score['S_jerk'], 50)
        self.assertAlmostEqual(score['S_a'], 100)
        self.assertAlmostEqual(score['MTSI'], 62.5)

    def test_both_sensors_contribute(self):
        analyzer = MTSIAnalyzer()
        for k in range(100):
            score = analyzer.update(dict(timestamp=k/100, omega=[0, 0],
                                         acceleration=[0, 0], jerk=[0, 12]))
        self.assertAlmostEqual(score['S_jerk'], 100/1.5)

    def test_invalid_inputs_and_nonincreasing_time(self):
        for kwargs in ({'weights': (1, 1, 1)}, {'scales': (0, 1, 1)},
                       {'scales': (math.nan, 1, 1)}, {'window_samples': 1}):
            with self.assertRaises(ValueError):
                MTSIAnalyzer(**kwargs)
        with self.assertRaises(ValueError):
            IMUPreprocessor(direction_unit=(0, 0, 0))
        pipeline = DualIMUPipeline()
        self.stationary(pipeline, 1)
        with self.assertRaises(ValueError):
            self.stationary(pipeline, 1)
        with self.assertRaises(ValueError):
            pipeline.update(2, [GRAVITY]*2, [(0, 0, 0)]*2, [(0, 0, 0, 0)]*2)
        # An invalid input must not advance preprocessing state.
        self.assertEqual(pipeline.preprocessor.previous_timestamp, 1)

    def test_demo_abrupt_translation_has_lower_score(self):
        pipeline = DualIMUPipeline()
        groups = {}
        for row in demo_records():
            _, score = pipeline.update(*(row[k] for k in
                ('timestamp', 'accelerations', 'gyros', 'quaternions')))
            if score and row['timestamp'] % 4 >= 2:
                groups.setdefault(row['label'], []).append(score['MTSI'])
        averages = {key: sum(v)/len(v) for key, v in groups.items()}
        self.assertEqual(averages['stationary'], 100)
        self.assertGreater(averages['smooth_translation'], averages['abrupt_translation'])


if __name__ == '__main__':
    unittest.main()
