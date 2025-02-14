package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private SparkMax outtakeMotor;
  private SparkClosedLoopController pid;
  private final SparkClosedLoopController outTakeController;
  private SparkMax outTakeSpark;
  private RelativeEncoder outTakeEncoder;
  private double setPoint = 0;
  private final boolean turnInverted = false;
  private final boolean turnEncoderInverted = false;
  private static final int turnMotorCurrentLimit = 30;
  private static final double turnEncoderPositionFactor = 1.0;
  private static final double turnEncoderVelocityFactor = 1.0;

  public Outtake() {

    // Configure turn motor
    outTakeController = outtakeMotor.getClosedLoopController();
    var outTakeConfig = new SparkMaxConfig();
    outTakeConfig
        .inverted(turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    outTakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(.01, 0, 0);
    tryUntilOk(
        outTakeSpark,
        5,
        () ->
            outTakeSpark.configure(
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
    return setPoint - outtakeMotor.getEncoder().getVelocity();
  }

  @AutoLogOutput(key = "outtake/atSpeed")
  private boolean atSpeed() {
    if (getError() < 100 && getError() > -100) {
      return true;
    } else return false;
  }

  public Command outtakeCoral() {
    return runOnce(
        () -> {
          setPoint = 7000;
        });
  }

  public Command placeCoral() {
    return runOnce(
        () -> {
          setPoint = 8000;
        });
  }

  public Command reverseOutTake() {
    return runOnce(
        () -> {
          setPoint = -7000;
        });
  }

  public Command pauseOutTake() {
    return runOnce(
        () -> {
          setPoint = 0;
        });
  }

  @Override
  public void periodic() {

    pid.setReference(setPoint, ControlType.kVelocity);

    Logger.recordOutput("outTake/Output", outtakeMotor.getAppliedOutput());
    Logger.recordOutput("outTake/speed", outtakeMotor.getEncoder().getVelocity());
    Logger.recordOutput("outTake/setPoint", setPoint);
  }
}
