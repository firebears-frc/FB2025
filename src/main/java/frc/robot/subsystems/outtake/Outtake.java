package frc.robot.subsystems.outtake;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  public static final class Constants {
    public static final double GEAR_RATIO = 9.0;
  }

  private final OuttakeIO io;
  private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final Debouncer debouncer = new Debouncer(0.2);

  @AutoLogOutput(key = "Outtake/Setpoint")
  private double setpoint = 0.0;

  public Outtake(OuttakeIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (frc.robot.Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Outtake", inputs);

    if (inputs.beamBrake) {
      stop();
    }
  }

  @AutoLogOutput(key = "Outtake/Speed")
  public double getSpeed() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadiansPerSecond);
  }

  @AutoLogOutput(key = "Outtake/Error")
  public double getError() {
    return getSpeed() - setpoint;
  }

  @AutoLogOutput(key = "Outtake/AtSpeed")
  public boolean atSpeed() {
    return Math.abs(getError()) < 100.0;
  }

  @AutoLogOutput(key = "Outtake/OnTarget")
  public boolean onTarget() {
    return debouncer.calculate(atSpeed());
  }

  /** Run closed loop at the specified velocity. */
  public void run(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    setpoint = velocityRPM;
  }

  /** Stops the outtake. */
  public void stop() {
    io.stop();
  }
}
