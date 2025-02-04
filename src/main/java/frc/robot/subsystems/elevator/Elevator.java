package frc.robot.subsystems.elevator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class Elevator extends SubsystemBase {
  private final SparkMax leftElevatorMotor = new SparkMax(10, MotorType.kBrushless);
  private SparkClosedLoopController leftClosedLoopController;
  private RelativeEncoder elevatorEncoder = leftElevatorMotor.getEncoder();

  private final SparkMax rightElevatorMotor = new SparkMax(11, MotorType.kBrushless);
  private SparkClosedLoopController rightClosedLoopController;

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

  public void elevatorSetpoint(Rotation2d setpoint) { // change values later
    if (setpoint.getDegrees() < -5) {
      setpoint = Rotation2d.fromDegrees(-5);
    } else if (setpoint.getDegrees() > 100) {
      setpoint = Rotation2d.fromDegrees(100);
    }
    elevatorSetpoint = setpoint;
  }

  @AutoLogOutput(key = "elevator/rotations")
  public Rotation2d getShoulderAngle() {
    return Rotation2d.fromRotations(elevatorEncoder.getPosition());
  }

  public void periodic() {}
}
