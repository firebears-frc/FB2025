package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public Rotation2d position = new Rotation2d();
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Run closed loop to the specified velocity. */
  public default void setPosition(Rotation2d angle, double ffVolts) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}
