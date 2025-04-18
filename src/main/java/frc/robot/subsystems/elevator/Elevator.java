package frc.robot.subsystems.elevator;

import static frc.robot.util.SparkUtil.tryUntilOk;

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
  private double setpointRotations = 0.0;

  public Elevator() {
    var leftSparkMaxConfig = new SparkMaxConfig();
    leftClosedLoopController = leftElevatorMotor.getClosedLoopController();

    leftSparkMaxConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40, 40)
        .secondaryCurrentLimit(50);

    leftSparkMaxConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.05, 0, 0)
        .maxOutput(1)
        .minOutput(-.374);

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
    setSetpointRotations(getPositionRotations());
  }

  private static final class constants { // arm setpoints
    private static final double zero = 0.0;
    private static final double pickUp = 0.0;
    private static final double levelOne = 14; // 12.7
    private static final double levelTwo = 29.7;
    private static final double levelThree = 47.4;
    private static final double levelFour = 72.0;

    private static final double elevetorG = 0.02;
  }

  public void setSetpointRotations(double setpointRotations) {
    if (setpointRotations < 0) {
      setpointRotations = 0;
    } else if (setpointRotations > 72) {
      setpointRotations = 72;
    }

    this.setpointRotations = setpointRotations;
  }

  @AutoLogOutput(key = "Elevator/Position")
  public double getPositionRotations() {
    return elevatorEncoder.getPosition();
  }

  @AutoLogOutput(key = "Elevator/Error")
  private double getErrorRotations() {
    return getPositionRotations() - setpointRotations;
  }

  private Command positionCommand(double positionRotations, double tolerance) {
    return Commands.sequence(
        runOnce(() -> setSetpointRotations(positionRotations)),
        Commands.waitSeconds(0.1),
        run(() -> {}).until(() -> onTarget(tolerance)));
  }

  private boolean onTarget(double toleranceRotations) {
    boolean onTarget = Math.abs(getErrorRotations()) < toleranceRotations;
    Logger.recordOutput("Elevator/On Target", onTarget);
    boolean debounced = debounce.calculate(onTarget);
    Logger.recordOutput("Elevator/On Target Debounced", debounced);
    return debounced;
  }

  public Command defaultCommand(Supplier<Double> elevatorChange) {
    return run(
        () -> {
          setSetpointRotations(setpointRotations - elevatorChange.get());
        });
  }

  public Command pickUp() {
    return positionCommand(constants.pickUp, 0.5);
  }

  public Command zero() {
    return positionCommand(constants.zero, 2);
  }

  public Command levelOne() {
    return positionCommand(constants.levelOne, 2.0);
  }

  public Command levelTwo() {
    return positionCommand(constants.levelTwo, 2.0);
  }

  public Command levelThree() {
    return positionCommand(constants.levelThree, 2.0);
  }

  public Command levelFour() {
    return positionCommand(constants.levelFour, 2.0);
  }

  public void periodic() {
    leftClosedLoopController.setReference(
        setpointRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0, constants.elevetorG);

    Logger.recordOutput("Elevator/MotorLeft", leftElevatorMotor.getAppliedOutput());
    Logger.recordOutput("Elevator/MotorRight", rightElevatorMotor.getAppliedOutput());
    Logger.recordOutput("Elevator/MotorLeftCurrent", leftElevatorMotor.getOutputCurrent());
    Logger.recordOutput("Elevator/MotorRightCurrent", rightElevatorMotor.getOutputCurrent());
  }
}
