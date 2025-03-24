package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  public static final class Constants {
    public static final double MINIMUM = 0.0;
    public static final double MAXIMUM = 79.0;
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final ElevatorFeedforward ffModel;
  private final Debouncer debouncer = new Debouncer(0.2);

  @AutoLogOutput(key = "Elevator/Setpoint")
  private double setpoint = 0.0;

  /** Creates a new Elevator. */
  public Elevator(ElevatorIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (frc.robot.Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new ElevatorFeedforward(0.0, 0.02, 0.0);
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new ElevatorFeedforward(0.0, 0.0, 0.0); // TODO
        io.configurePID(25.0, 0.0, 0.0);
        break;
      default:
        ffModel = new ElevatorFeedforward(0.0, 0.0, 0.0); // TODO
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    double ffVolts = ffModel.calculate(0.0);
    Logger.recordOutput("Elevator/ffVolts", ffVolts);
    io.setPosition(setpoint, ffVolts);
  }

  @AutoLogOutput(key = "Elevator/Error")
  public double getError() {
    return inputs.positionRotations - setpoint;
  }

  @AutoLogOutput(key = "Elevator/AtPosition")
  public boolean atPosition() {
    return Math.abs(getError()) < 1.0;
  }

  @AutoLogOutput(key = "Elevator/OnTarget")
  public boolean onTarget() {
    return debouncer.calculate(atPosition());
  }

  public void set(double rotations) {
    if (rotations > Constants.MAXIMUM) setpoint = Constants.MAXIMUM;
    else if (rotations < Constants.MINIMUM) setpoint = Constants.MINIMUM;
    else setpoint = rotations;
  }
}
