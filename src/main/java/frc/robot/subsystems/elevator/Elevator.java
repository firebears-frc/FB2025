package frc.robot.subsystems.elevator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final SparkMax leftElevatorMotor = new SparkMax(10, MotorType.kBrushless);
  private SparkClosedLoopController leftClosedLoopController;
  private RelativeEncoder elevatorEncoder = leftElevatorMotor.getEncoder();
  private final SparkMax rightElevatorMotor = new SparkMax(11, MotorType.kBrushless);
  private Debouncer debounce = new Debouncer(0.2);

  @AutoLogOutput(key = "Elevator/Setpoint")
  private Rotation2d elevatorSetpoint = new Rotation2d();

  public Elevator() {
    var leftSparkMaxConfig = new SparkMaxConfig();
    leftClosedLoopController = leftElevatorMotor.getClosedLoopController();

    leftSparkMaxConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40, 40)
        .secondaryCurrentLimit(50);

    leftSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.05, 0, 0);

    tryUntilOk(
        leftElevatorMotor,
        5,
        () ->
            leftElevatorMotor.configure(
                leftSparkMaxConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    var rightSparkMaxConfig = new SparkMaxConfig();
    rightSparkMaxConfig.follow(10, true);

    tryUntilOk(
        rightElevatorMotor,
        5,
        () ->
            rightElevatorMotor.configure(
                rightSparkMaxConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    setElevatorSetpoint(getElevatorRotations());
  }

  private static final class constants { // arm setpoints
    private static final Rotation2d zero = Rotation2d.fromRotations(0.0);
    private static final Rotation2d pickUp = Rotation2d.fromRotations(0.0);
    private static final Rotation2d levelOne = Rotation2d.fromRotations(10.0);
    private static final Rotation2d levelTwo = Rotation2d.fromRotations(20.0);
    private static final Rotation2d levelThree = Rotation2d.fromRotations(30.0);
    private static final Rotation2d levelFour = Rotation2d.fromRotations(40.0);

    private static final double elevetorG = 0.02;
  }

  public void setElevatorSetpoint(Rotation2d setpoint) {
    /*  if (setpoint.getDegrees() < -5) {
      setpoint = Rotation2d.fromDegrees(-5);
    } else if (setpoint.getDegrees() > 100) {
      setpoint = Rotation2d.fromDegrees(100);
    }
    */
    elevatorSetpoint = setpoint;
  }

  @AutoLogOutput(key = "Elevator/Position")
  public Rotation2d getElevatorRotations() {
    return Rotation2d.fromRotations(elevatorEncoder.getPosition());
  }

  @AutoLogOutput(key = "Elevator/Error")
  private Rotation2d getError() {
    Rotation2d position = getElevatorRotations();
    Logger.recordOutput("Elevator/Debug/Position", position);
    Rotation2d setpoint = elevatorSetpoint;
    Logger.recordOutput("Elevator/Debug/Setpoint", setpoint);
    Rotation2d error = position.minus(setpoint);
    Logger.recordOutput("Elevator/Debug/Error", error);
    System.out.println(
        "getError(): "
            + position.getRotations()
            + " - "
            + setpoint.getRotations()
            + " = "
            + error.getRotations());
    return error;
  }

  private Command positionCommand(Rotation2d position, Rotation2d tolerance) {
    return Commands.sequence(
        runOnce(() -> setElevatorSetpoint(position)),
        Commands.waitSeconds(0.1),
        run(() -> {}).until(() -> onTarget(tolerance)));
  }

  private boolean onTarget(Rotation2d tolerance) {
    Logger.recordOutput("Elevator/Debug/Tolerance", tolerance);
    boolean onTarget = Math.abs(getError().getRotations()) < tolerance.getRotations();
    Logger.recordOutput("Elevator/Debug/On Target", onTarget);
    boolean debounced = debounce.calculate(onTarget);
    Logger.recordOutput("Elevator/Debug/Debounced On Target", debounced);
    System.out.println(
        "onTarget(): " + tolerance.getRotations() + " < " + onTarget + ", " + debounced);
    return debounced;
  }

  public Command defaultCommand(Supplier<Double> elevatorChange) {
    return run(
        () -> {
          setElevatorSetpoint(
              elevatorSetpoint.minus(Rotation2d.fromRotations(elevatorChange.get())));
        });
  }

  public Command pickUp() {
    return positionCommand(constants.pickUp, Rotation2d.fromRotations(0.5));
  }

  public Command zero() {
    return positionCommand(constants.zero, Rotation2d.fromRotations(0.5));
  }

  public Command levelOne() {
    return positionCommand(constants.levelOne, Rotation2d.fromRotations(0.5));
  }

  public Command levelTwo() {
    return positionCommand(constants.levelTwo, Rotation2d.fromRotations(0.5));
  }

  public Command levelThree() {
    return positionCommand(constants.levelThree, Rotation2d.fromRotations(0.5));
  }

  public Command levelFour() {
    return positionCommand(constants.levelFour, Rotation2d.fromRotations(0.5));
  }

  public void periodic() {
    leftClosedLoopController.setReference(
        elevatorSetpoint.getRotations(),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        constants.elevetorG);

    Logger.recordOutput("Elevator/MotorLeft", leftElevatorMotor.getAppliedOutput());
    Logger.recordOutput("Elevator/MotorRight", rightElevatorMotor.getAppliedOutput());
    Logger.recordOutput("Elevator/MotorLeftCurrent", leftElevatorMotor.getOutputCurrent());
    Logger.recordOutput("Elevator/MotorRightCurrent", rightElevatorMotor.getOutputCurrent());
  }
}
