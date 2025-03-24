package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  public static final class Constants {
    public static final double GEAR_RATIO = 144.0;

    public static final Rotation2d MINIMUM = Rotation2d.fromDegrees(-10.0);
    public static final Rotation2d MAXIMUM = Rotation2d.fromDegrees(120.0);
  }

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final ArmFeedforward ffModel;
  private final Debouncer debouncer = new Debouncer(0.2);

  @AutoLogOutput(key = "Climber/Setpoint")
  private Rotation2d setpoint = null;

  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (frc.robot.Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new ArmFeedforward(0.1, 0.35, 3.74, 0.02);
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new ArmFeedforward(0.0, 0.35, 3.74, 0.02);
        io.configurePID(25.0, 0.0, 0.0);
        break;
      default:
        ffModel = new ArmFeedforward(0.0, 0.0, 0.0);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    if (setpoint == null) setpoint = inputs.position;

    double ffVolts = ffModel.calculate(inputs.position.getRadians(), 0.0);
    Logger.recordOutput("Climber/ffVolts", ffVolts);
    io.setPosition(setpoint, ffVolts);
  }

  @AutoLogOutput(key = "Climber/Error")
  public Rotation2d getError() {
    return inputs.position.minus(setpoint);
  }

  @AutoLogOutput(key = "Climber/AtPosition")
  public boolean atPosition() {
    return Math.abs(getError().getDegrees()) < 1.0;
  }

  @AutoLogOutput(key = "Climber/OnTarget")
  public boolean onTarget() {
    return debouncer.calculate(atPosition());
  }

  public void set(Rotation2d angle) {
    if (angle.getDegrees() > Constants.MAXIMUM.getDegrees()) setpoint = Constants.MAXIMUM;
    else if (angle.getDegrees() < Constants.MINIMUM.getDegrees()) setpoint = Constants.MINIMUM;
    else setpoint = angle;
  }
}
