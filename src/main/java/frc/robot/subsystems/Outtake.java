package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private SparkMax outtakeMotor = new SparkMax(12, MotorType.kBrushless);
  private final SparkClosedLoopController outTakeController;
  private RelativeEncoder encoder;
  private double setPoint = 0; // velocity
  private double startPosition = 0;
  private double setPosition = 0;
  private static final int outtakeCurrentLimit = 30;

  public Outtake() {

    // Configure turn motor
    outTakeController = outtakeMotor.getClosedLoopController();
    var outTakeConfig = new SparkMaxConfig();
    outTakeConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(outtakeCurrentLimit)
        .secondaryCurrentLimit(50)
        .voltageCompensation(12.0);
    outTakeConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(.00007, 0.0, 0.0, 1.774691358024691e-4);
    // ff: 1.774691358024691e-4

    encoder = outtakeMotor.getEncoder();
    startPosition = setPosition = encoder.getPosition();

    tryUntilOk(
        outtakeMotor,
        5,
        () ->
            outtakeMotor.configure(
                outTakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  private void tryUntilOk(SparkMax spark, int retries, Runnable config) {
    for (int i = 0; i < retries; i++) {
      try {
        config.run();
        break;
      } catch (Exception e) {
        if (i == retries - 1) {
          throw e;
        }
      }
    }
  }

  @AutoLogOutput(key = "outtake/error")
  private double getError() {
    return setPoint - encoder.getVelocity();
  }

  @AutoLogOutput(key = "outtake/atSpeed")
  private boolean atSpeed() {
    return getError() < 100 && getError() > -100;
  }

  public Command outtakeCoral() {
    return runOnce(
        () -> {
          setPoint = 5000;
        });
  }

  public Command placeCoral() {
    return runOnce(
        () -> {
          setPoint = 5000;
        });
  }

  public Command reverseOutTake() {
    return runOnce(
        () -> {
          setPoint = -5000;
        });
  }

  public Command pauseOutTake() {
    return runOnce(
        () -> {
          setPoint = 0;
        });
  }

  public Command incrementPosition() {
    return runOnce(
        () -> {
          setPosition += 100;
        });
  }

  public Command decrementPosition() {
    return runOnce(
        () -> {
          double newPosition = setPosition - 100;
          setPosition = startPosition > newPosition ? startPosition : newPosition;
        });
  }

  @Override
  public void periodic() {

    outTakeController.setReference(setPoint, ControlType.kVelocity);

    Logger.recordOutput("outTake/Output", outtakeMotor.getAppliedOutput());
    Logger.recordOutput("outTake/speed", encoder.getVelocity());
    Logger.recordOutput("outTake/setPoint", setPoint);
  }
}
