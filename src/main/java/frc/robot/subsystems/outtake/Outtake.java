package frc.robot.subsystems.outtake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Outtake extends SubsystemBase {
  public static final class Constants {
    public static final double GEAR_RATIO = 9.0;
  }

  private static enum State {
    STOPPED,
    EJECT,
    OUT,
    IN,
    REVERSE,
    SYSID
  }

  private final LoggedNetworkNumber ejectSpeed =
      new LoggedNetworkNumber("SmartDashboard/Outtake/Eject Speed", 775);
  private final LoggedNetworkNumber outSpeed =
      new LoggedNetworkNumber("SmartDashboard/Outtake/Out Speed", 550);
  private final LoggedNetworkNumber inSpeed =
      new LoggedNetworkNumber("SmartDashboard/Outtake/In Speed", 250);
  private final LoggedNetworkNumber reverseSpeed =
      new LoggedNetworkNumber("SmartDashboard/Outtake/Reverse Speed", -250);

  private final OuttakeIO io;
  private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final Debouncer debouncer = new Debouncer(0.2);
  private final SysIdRoutine sysId;

  @AutoLogOutput(key = "Outtake/State")
  private State state = State.STOPPED;

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

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Outtake/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Outtake", inputs);

    switch (state) {
      case STOPPED:
        stopOuttake();
        break;
      case EJECT:
        runVelocity(ejectSpeed.get());
        break;
      case OUT:
        runVelocity(outSpeed.get());
        break;
      case IN:
        runVelocity(inSpeed.get());
        break;
      case REVERSE:
        runVelocity(reverseSpeed.get());
        break;
      case SYSID:
        break;
    }
  }

  @AutoLogOutput(key = "Outtake/Speed")
  private double getSpeed() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadiansPerSecond);
  }

  @AutoLogOutput(key = "Outtake/Error")
  private double getError() {
    return getSpeed() - setpoint;
  }

  @AutoLogOutput(key = "Outtake/AtSpeed")
  private boolean atSpeed() {
    return Math.abs(getError()) < 100.0;
  }

  @AutoLogOutput(key = "Outtake/OnTarget")
  private boolean onTarget() {
    return debouncer.calculate(atSpeed());
  }

  /** Run open loop at the specified voltage. */
  private void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  private void runVelocity(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    setpoint = velocityRPM;
  }

  /** Stops the outtake. */
  private void stopOuttake() {
    state = State.STOPPED;
    io.stop();
  }

  /** Returns a command to run the outtake at a set state. */
  private Command stateCommand(State state) {
    return Commands.sequence(
        runOnce(() -> this.state = state),
        Commands.waitSeconds(0.25),
        run(() -> {}).until(this::onTarget));
  }

  /** Returns a command to stop the outtake. */
  public Command stop() {
    return runOnce(this::stopOuttake);
  }

  /** Returns a command to run the outtake at eject state. */
  public Command eject() {
    return stateCommand(State.EJECT);
  }

  /** Returns a command to run the outtake at out state. */
  public Command out() {
    return stateCommand(State.OUT);
  }

  /** Returns a command to run the outtake at in state. */
  public Command in() {
    return stateCommand(State.IN);
  }

  /** Returns a command to run the outtake at reverse state. */
  public Command reverse() {
    return stateCommand(State.REVERSE);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.sequence(
        runOnce(() -> state = State.SYSID), sysId.quasistatic(direction), stop());
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.sequence(runOnce(() -> state = State.SYSID), sysId.dynamic(direction), stop());
  }
}
