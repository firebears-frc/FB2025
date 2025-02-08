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
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final SparkMax leftElevatorMotor = new SparkMax(10, MotorType.kBrushless);
  private SparkClosedLoopController leftClosedLoopController;
  private RelativeEncoder elevatorEncoder = leftElevatorMotor.getEncoder();

  private final SparkMax rightElevatorMotor = new SparkMax(11, MotorType.kBrushless);
  private SparkClosedLoopController rightClosedLoopController;

  private Debouncer debounce = new Debouncer(0.2);

  @AutoLogOutput(key = "elevator/setPoint")
  private Rotation2d elevatorSetpoint = new Rotation2d();

  public Elevator() {
    var leftSparkMaxConfig = new SparkMaxConfig();
    leftClosedLoopController = leftElevatorMotor.getClosedLoopController();

    leftSparkMaxConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40, 40)
        .secondaryCurrentLimit(50);

    leftSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0, 0, 0);

    tryUntilOk(
        leftElevatorMotor,
        5,
        () ->
            leftElevatorMotor.configure(
                leftSparkMaxConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    var rightSparkMaxConfig = new SparkMaxConfig();
    rightClosedLoopController = rightElevatorMotor.getClosedLoopController();

    rightSparkMaxConfig.follow(10, true);

    tryUntilOk(
        rightElevatorMotor,
        5,
        () ->
            rightElevatorMotor.configure(
                rightSparkMaxConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  private static final class constants { // arm setpoints
    private static final Rotation2d zero = Rotation2d.fromRotations(0.0);
    private static final Rotation2d pickUp = Rotation2d.fromRotations(0.0);
    private static final Rotation2d levelOne = Rotation2d.fromRotations(0.0);
    private static final Rotation2d levelTwo = Rotation2d.fromRotations(0.0);
    private static final Rotation2d levelThree = Rotation2d.fromRotations(0.0);
    private static final Rotation2d levelFour = Rotation2d.fromRotations(0.0);

    private static final double elevetorG = 0.0;
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

  @AutoLogOutput(key = "elevator/rotations")
  public Rotation2d getElevatorRotations() {
    return Rotation2d.fromRotations(elevatorEncoder.getPosition());
  }

  @AutoLogOutput(key = "elevator/error")
  private Rotation2d getError() {
    return getElevatorRotations().minus(elevatorSetpoint);
  }

  private Command positionCommand(Rotation2d position, double tolerance) {
    return Commands.sequence(
        runOnce(() -> setElevatorSetpoint(position)),
        Commands.waitSeconds(0.1),
        run(() -> {}).until(() -> onTarget(tolerance)));
  }

  private boolean onTarget(double tolerance) {
    boolean onTarget = Math.abs(getError().getDegrees()) < tolerance;
    Logger.recordOutput("elevator/onTargt", onTarget);
    boolean debounced = debounce.calculate(onTarget);
    Logger.recordOutput("elevator/at debouncespeed", debounced);
    return debounced;
  }

  public Command pickUp() {
    return positionCommand(constants.pickUp, 1);
  }

  public Command zero() {
    return positionCommand(constants.zero, 1);
  }

  public Command levelOne() {
    return positionCommand(constants.levelOne, 1);
  }

  public Command levelTwo() {
    return positionCommand(constants.levelTwo, 1);
  }

  public Command levelThree() {
    return positionCommand(constants.levelThree, 1);
  }

  public Command levelFour() {
    return positionCommand(constants.levelFour, 1);
  }

  public void periodic() {
    leftClosedLoopController.setReference(
        elevatorSetpoint.getRotations(),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        constants.elevetorG);

    Logger.recordOutput("elevator/MotorLeft", leftElevatorMotor.getAppliedOutput());
    Logger.recordOutput("elevator/MotorRight", rightElevatorMotor.getAppliedOutput());
    Logger.recordOutput("elevator/MotorLeftCurrent", leftElevatorMotor.getOutputCurrent());
    Logger.recordOutput("elevator/MotorRightCurrent", rightElevatorMotor.getOutputCurrent());
    Logger.recordOutput("elevator/setPointDegrees", elevatorSetpoint.getRotations());
  }
}
