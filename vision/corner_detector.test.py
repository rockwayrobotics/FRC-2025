import unittest
import corner_detector as cd

class TestCornerDetector(unittest.TestCase):
  def test_simple(self):
    detector = cd.CornerDetector(4)
    detector.add_record(1., 301.)
    detector.add_record(2., 302.)
    detector.add_record(3., 303.)
    detector.add_record(4., 304.)
    detector.add_record(5., 305.)
    detector.add_record(6., 306.)
    detector.add_record(7., 307.)
    detector.add_record(8., 308.)
    detector.add_record(9., 309.)
    detector.add_record(10., 299.)
    detector.add_record(11., 289.)
    detector.add_record(12., 279.)
    detector.add_record(13., 269.)
    detector.add_record(14., 259.)
    detector.add_record(15., 249.)
    self.assertTrue(detector.found_corner())
    self.assertEqual(detector.corner_timestamp, 9)

  def test_manual(self):
    detector = cd.CornerDetector(10)
    detector.add_record(1., 287.)
    detector.add_record(2., 265.)
    detector.add_record(3., 244.)
    detector.add_record(4., 221.)
    detector.add_record(5., 200.)
    detector.add_record(6., 180.)
    detector.add_record(7., 178.)
    detector.add_record(8., 177.)
    detector.add_record(9., 175.)
    detector.add_record(10., 174.)
    detector.add_record(11., 172.)
    detector.add_record(12., 170.)
    detector.add_record(13., 169.)
    detector.add_record(14., 168.)
    detector.add_record(15., 166.)
    self.assertTrue(detector.found_corner())
    self.assertAlmostEqual(detector.corner_timestamp, 6, 1)

if __name__ == '__main__':
  unittest.main()

