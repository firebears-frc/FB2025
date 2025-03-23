package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Climber extends SubsystemBase {
  public static final class Constants {
    public static final double GEAR_RATIO = 144.0;

    public static final Rotation2d MINIMUM = Rotation2d.fromDegrees(-10.0);
    public static final Rotation2d MAXIMUM = Rotation2d.fromDegrees(120.0);
  }

  private static enum State {
    STARTUP,
    STOW,
    GRAB,
    CLIMB,
    SYSID
  }

  private final LoggedNetworkNumber stowInput = new LoggedNetworkNumber("Climber/Stow Angle", 0.0);
  private final LoggedNetworkNumber grabInput = new LoggedNetworkNumber("Climber/Grab Angle", 90.0);
  private final LoggedNetworkNumber climbInput =
      new LoggedNetworkNumber("Climber/Climb Angle", -15.0);

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final ArmFeedforward ffModel;
  private final Debouncer debouncer = new Debouncer(0.2);
  private final SysIdRoutine sysId;

  @AutoLogOutput(key = "Climber/State")
  private State state = State.STARTUP;

  @AutoLogOutput(key = "Climber/Setpoint")
  private Rotation2d setpoint;

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

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Climber/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    switch (state) {
      case STARTUP:
        // On init, set the setpoint to the current position
        if (setpoint == null) setAngle(inputs.position);
        break;
      case STOW:
        setAngle(Rotation2d.fromDegrees(stowInput.get()));
        break;
      case GRAB:
        setAngle(Rotation2d.fromDegrees(grabInput.get()));
        break;
      case CLIMB:
        setAngle(Rotation2d.fromDegrees(climbInput.get()));
        break;
      case SYSID:
        // TODO
        break;
    }

    double ffVolts = ffModel.calculate(inputs.position.getRadians(), 0.0);
    Logger.recordOutput("Climber/ffVolts", ffVolts);
    io.setPosition(setpoint, ffVolts);
  }

  @AutoLogOutput(key = "Climber/Error")
  private Rotation2d getError() {
    return inputs.position.minus(setpoint);
  }

  @AutoLogOutput(key = "Climber/AtPosition")
  private boolean atPosition() {
    return Math.abs(getError().getDegrees()) < 1.0;
  }

  @AutoLogOutput(key = "Climber/OnTarget")
  private boolean onTarget() {
    return debouncer.calculate(atPosition());
  }

  /** Run open loop at the specified voltage. */
  private void runVolts(double volts) {
    io.setVoltage(volts);
  }

  private void setAngle(Rotation2d angle) {
    if (angle.getDegrees() > Constants.MAXIMUM.getDegrees()) setpoint = Constants.MAXIMUM;
    else if (angle.getDegrees() < Constants.MINIMUM.getDegrees()) setpoint = Constants.MINIMUM;
    else setpoint = angle;
  }

  /** Returns a command to move the climber to the specified state. */
  private Command stateCommand(State state) {
    return Commands.sequence(
        runOnce(() -> this.state = state),
        Commands.waitSeconds(0.25),
        run(() -> {}).until(this::onTarget));
  }

  /** Returns a command to move the Climber to stow state. */
  public Command stow() {
    return stateCommand(State.STOW);
  }

  /** Returns a command to move the Climber to grab state. */
  public Command grab() {
    return stateCommand(State.GRAB);
  }

  /** Returns a command to move the Climber to climb state. */
  public Command climb() {
    return stateCommand(State.CLIMB);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.sequence(
        runOnce(() -> state = State.SYSID), sysId.quasistatic(direction), stow());
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.sequence(runOnce(() -> state = State.SYSID), sysId.dynamic(direction), stow());
  }
}
