package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AnglerPIDSubsystem;

public class ShooterAngle extends Command {

  private final AnglerPIDSubsystem m_angler;
  private double m_maxSpeed;
  private double m_distance;
  // private ScoringTarget targetAngle;
  // private ScoringTarget m_angle;

  private final PIDController pid;

  public ShooterAngle(AnglerPIDSubsystem angler, double maxSpeed) {
    m_angler = angler;
    m_maxSpeed = maxSpeed;

    addRequirements(m_angler);

    pid = new PIDController(.02, 0, 0);
    pid.setTolerance(Constants.Angler.ANGLE_BOTTOM_MAX / 25);
  }

  @Override
  public void initialize() {
    // switch (m_angler.m_ScoringMode) {
    //   case SPEAKER -> m_distance = m_angler.speakerAngleSetpoint;
    //   case AMP -> m_distance = m_angler.ampAngleSetpoint;
    // }
    // ;

    pid.setSetpoint(m_distance);
    pid.reset();
  }

  @Override
  public void execute() {
    double currentAngle = m_angler.getAngleEncoder();

    double spinPower = pid.calculate(currentAngle);

    spinPower = MathUtil.clamp(spinPower, -m_maxSpeed, m_maxSpeed);

    m_angler.setAngleMotor(spinPower);
  }

  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }

  @Override
  public void end(boolean cancelled) {
    System.out.println("Angled to: " + m_angler.m_ScoringMode);
    m_angler.setAngleMotor(0); // Resets the angle motor to 0, ends command
  }
}
