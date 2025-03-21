package frc.robot.util;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;

public class REVUtils {
  // See https://www.revrobotics.com/rev-21-1652/
  public static final double NEO_FF = 1.0 / 485.0;
  public static final double VORTEX_FF = 1.0 / 559.0;

  private static int failureCount = 0;

  public static int getFailureCount() {
    return failureCount;
  }

  public static void resetFailureCount() {
    failureCount = 0;
  }

  private static void recordFailure() {
    failureCount++;
    Logger.recordOutput("RevUtils/failure_count", failureCount);
  }

  /**
   * REVLib motors have error codes that are returned when commands fail.
   * The question is what should we do when they fail? Currently we just try
   * 5 times and then give up.
   */
  public static void tryUntilOk(Supplier<REVLibError> command) {
    for (int i = 0; i < 5; i++) {
      REVLibError error = command.get();
      if (error == REVLibError.kOk) {
        break;
      } else {
        recordFailure();
      }
    }
  }

  /**
   * REVLib motors have error codes that are returned when getters fail.
   * Avoid setting the inputs if the getter fails.
   */
  public static void ifOk(SparkBase motor, DoubleSupplier supplier, DoubleConsumer consumer) {
    double value = supplier.getAsDouble();
    if (motor.getLastError() == REVLibError.kOk) {
      consumer.accept(value);
    } else {
      recordFailure();
    }
  }

  /**
   * REVLib motors have error codes that are returned when getters fail.
   * Avoid setting the inputs if the getter fails.
   */
  public static void ifOk(SparkBase motor, DoubleSupplier[] suppliers, Consumer<double[]> consumer) {
    double[] values = new double[suppliers.length];
    for (int i = 0; i < suppliers.length; i++) {
      values[i] = suppliers[i].getAsDouble();
      if (motor.getLastError() != REVLibError.kOk) {
        recordFailure();
        return;
      }
    }
    consumer.accept(values);
  }
}
