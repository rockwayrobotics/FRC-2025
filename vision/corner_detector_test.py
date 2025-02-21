import unittest
import corner_detector as cd


class TestCornerDetector(unittest.TestCase):
    def test_shift(self):
        detector = cd.CornerDetector(1)
        for i in range(1, 999):
            detector.add_record(float(i), 200.0 + float(i) / 10.0)
        self.assertEqual(detector.start_index, 984)
        self.assertEqual(detector.end_index, 998)
        detector.add_record(999.0, 200.0 + float(999.0) / 10.0)
        self.assertEqual(detector.start_index, 985)
        self.assertEqual(detector.end_index, 999)
        detector.add_record(1000.0, 200.0 + float(1000.0) / 10.0)
        self.assertEqual(detector.start_index, 1)
        self.assertEqual(detector.end_index, 16)
        detector.add_record(1001.0, 200.0 + float(1001.0) / 10.0)
        self.assertEqual(detector.start_index, 2)
        self.assertEqual(detector.end_index, 17)

    def test_simple(self):
        detector = cd.CornerDetector(4)
        for i in range(0, 10):
            detector.add_record(float(i + 1), 450.0 - 30 * i)
        for i in range(0, 10):
            detector.add_record(float(i + 11), 150.0 + 30 * i)
        self.assertTrue(detector.found_corner())
        self.assertEqual(detector.corner_timestamp, 11)

    def test_manual(self):
        detector = cd.CornerDetector(10)
        detector.add_record(1.0, 287.0)
        detector.add_record(2.0, 265.0)
        detector.add_record(3.0, 244.0)
        detector.add_record(4.0, 221.0)
        detector.add_record(5.0, 200.0)
        detector.add_record(6.0, 180.0)
        detector.add_record(7.0, 178.0)
        detector.add_record(8.0, 177.0)
        detector.add_record(9.0, 175.0)
        detector.add_record(10.0, 174.0)
        detector.add_record(11.0, 172.0)
        detector.add_record(12.0, 170.0)
        detector.add_record(13.0, 169.0)
        detector.add_record(14.0, 168.0)
        detector.add_record(15.0, 166.0)
        self.assertTrue(detector.found_corner())
        self.assertAlmostEqual(detector.corner_timestamp, 6, 1)

    def test_ignore_spike(self):
        detector = cd.CornerDetector(10)
        detector.add_record(1.0, 287.0)
        detector.add_record(2.0, 265.0)
        detector.add_record(3.0, 244.0)
        detector.add_record(4.0, 221.0)
        detector.add_record(5.0, 200.0)
        detector.add_record(6.0, 180.0)
        detector.add_record(7.0, 478.0)
        detector.add_record(8.0, 177.0)
        detector.add_record(9.0, 175.0)
        detector.add_record(10.0, 174.0)
        detector.add_record(11.0, 172.0)
        detector.add_record(12.0, 170.0)
        detector.add_record(13.0, 169.0)
        detector.add_record(14.0, 168.0)
        detector.add_record(15.0, 166.0)
        self.assertTrue(detector.found_corner())
        self.assertAlmostEqual(detector.corner_timestamp, 6, 1)

    def test_ignore_below_min(self):
        detector = cd.CornerDetector(10)
        detector.add_record(1.0, 287.0)
        detector.add_record(2.0, 265.0)
        detector.add_record(3.0, 244.0)
        detector.add_record(4.0, 221.0)
        detector.add_record(5.0, 200.0)
        detector.add_record(6.0, 180.0)
        detector.add_record(7.0, 0.0)
        detector.add_record(8.0, 177.0)
        detector.add_record(9.0, 175.0)
        detector.add_record(10.0, 174.0)
        detector.add_record(11.0, 172.0)
        detector.add_record(12.0, 170.0)
        detector.add_record(13.0, 169.0)
        detector.add_record(14.0, 168.0)
        detector.add_record(15.0, 166.0)
        detector.add_record(16.0, 165.0)
        self.assertTrue(detector.found_corner())
        self.assertAlmostEqual(detector.corner_timestamp, 6, 1)

    def test_reset_beyond_max(self):
        detector = cd.CornerDetector(10)
        detector.add_record(1.0, 287.0)
        detector.add_record(2.0, 265.0)
        detector.add_record(3.0, 244.0)
        detector.add_record(4.0, 221.0)
        detector.add_record(5.0, 200.0)
        detector.add_record(6.0, 180.0)
        detector.add_record(7.0, 778.0)
        detector.add_record(8.0, 177.0)
        detector.add_record(9.0, 175.0)
        detector.add_record(10.0, 174.0)
        detector.add_record(11.0, 172.0)
        detector.add_record(12.0, 170.0)
        detector.add_record(13.0, 169.0)
        detector.add_record(14.0, 168.0)
        detector.add_record(15.0, 166.0)
        detector.add_record(16.0, 165.0)
        self.assertFalse(detector.found_corner())


if __name__ == "__main__":
    unittest.main()
