package frc.robot.subsystems.climp;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;
import frc.robot.util.REVUtils;
import frc.robot.util.Tuner;

public class ClimpIOReal implements ClimpIO {
  protected final SparkFlex climpMotor = new SparkFlex(Constants.CAN.CLIMP_MOTOR, MotorType.kBrushless);

  final Tuner pid_p = new Tuner("Climp/Kp", 0, true);
  final Tuner pid_d = new Tuner("Climp/Kd", 0, true);
  final Tuner currentLimit = new Tuner("Climp/current_limit", 5.0, true);
  final Tuner maxNormalizedSpeed = new Tuner("Climp/normalized_speed_max", 0.1, true);
  final Tuner minNormalizedSpeed = new Tuner("Climp/normalized_speed_min", -0.1, true);

  protected final RelativeEncoder encoder = climpMotor.getEncoder();

  public ClimpIOReal() {
    updateParams(true);

    pid_p.addListener((_e) -> updateParams(false));
    pid_d.addListener((_e) -> updateParams(false));
    currentLimit.addListener((_e) -> updateParams(false));
    maxNormalizedSpeed.addListener((_e) -> updateParams(false));
    minNormalizedSpeed.addListener((_e) -> updateParams(false));
  }

  @Override
  public void updateInputs(ClimpIOInputs inputs) {
    REVUtils.ifOk(climpMotor, encoder::getPosition, (value) -> inputs.angleRadians = value);
    REVUtils.ifOk(climpMotor, encoder::getVelocity, (value) -> inputs.velocityRadsPerSec = value);
    REVUtils.ifOk(climpMotor, new DoubleSupplier[] {
        climpMotor::getAppliedOutput, climpMotor::getBusVoltage
    }, (values) -> inputs.appliedVoltage = values[0] * values[1]);
    REVUtils.ifOk(climpMotor, climpMotor::getOutputCurrent, (value) -> inputs.supplyCurrentAmps = value);
  }

  @Override
  public void setNormalizedSpeed(double speed) {
    climpMotor.set(speed);
  }

  @Override
  public void stop() {
    climpMotor.set(0);
  }

  @Override
  public void moveTowardsGoal(double goalPositionMeters, double currentPositionMeters) {
    double error = goalPositionMeters - currentPositionMeters;
    double speed = error * pid_p.get();
    climpMotor.set(speed);
  }

  private void updateParams(boolean resetSafe) {
    ResetMode resetMode = resetSafe ? ResetMode.kResetSafeParameters : ResetMode.kNoResetSafeParameters;
    SparkFlexConfig config = new SparkFlexConfig();
    if (resetSafe) {
      config.idleMode(IdleMode.kBrake).voltageCompensation(12.0);
      config.encoder.positionConversionFactor(2 * Math.PI / Constants.Climp.PIVOT_GEAR_RATIO)
          .velocityConversionFactor(2 * Math.PI / Constants.Climp.PIVOT_GEAR_RATIO / 60);
    }
    config.smartCurrentLimit((int) currentLimit.get());
    config.closedLoop.pidf(pid_p.get(), 0, pid_d.get(), 0);
    config.closedLoop.outputRange(minNormalizedSpeed.get(), maxNormalizedSpeed.get());

    REVUtils.tryUntilOk(() -> climpMotor.configure(config, resetMode, PersistMode.kPersistParameters));

  }
}
