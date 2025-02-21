package frc.robot.util;

import java.util.LinkedList;

public class TimestampSynchronizer {
  private final int windowSize;
  private final LinkedList<SyncSample> samples = new LinkedList<>();

  private static class SyncSample {
    final double offset;

    SyncSample(double monotonic, double ntTimestamp) {
      this.offset = ntTimestamp - monotonic;
    }
  }

  /**
   * Create a timestamp synchronizer with the default window size of 10.
   */
  public TimestampSynchronizer() {
    this(10);
  }

  /**
   * Create a timestamp synchronizer that receives time pairs (monotonic, fpga)
   * and can convert monotonic time back to fpga time with the offset.
   * @param windowSize Number of samples used for average
   */
  public TimestampSynchronizer(int windowSize) {
    this.windowSize = windowSize;
  }

  public void addTimes(double monotonic, double ntTimestamp) {
    samples.add(new SyncSample(monotonic, ntTimestamp));
    if (samples.size() > this.windowSize) {
      samples.removeFirst();
    }
  }

  public double toFPGATimestamp(double monotonic, double ifNoData) {
    if (samples.isEmpty()) {
      return ifNoData;
    }

    double[] offsets = samples.stream().mapToDouble(sample -> sample.offset).sorted().toArray();
    double medianOffset;
    if (offsets.length % 2 == 0) {
      medianOffset = (offsets[offsets.length / 2] + offsets[offsets.length / 2 - 1]) / 2;
    } else {
      medianOffset = offsets[offsets.length / 2];
    }
    return monotonic - medianOffset;
  }
}
