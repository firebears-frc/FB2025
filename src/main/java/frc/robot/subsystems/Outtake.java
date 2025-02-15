package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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

public class Outtake extends SubsystemBase {
  private SparkMax outtakeMotor = new SparkMax(12, MotorType.kBrushless);
  private final SparkClosedLoopController outTakeController;
  private RelativeEncoder outTakeEncoder;
  private double setPoint = 0;
  private static final int outtakeCurrentLimit = 30;

  public Outtake() {

    // Configure turn motor
    outTakeController = outtakeMotor.getClosedLoopController();
    var outTakeConfig = new SparkMaxConfig();
    outTakeConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(outtakeCurrentLimit)
        .secondaryCurrentLimit(50)
        .voltageCompensation(12.0);
    outTakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(.01, 0, 0);
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
    return setPoint - outtakeMotor.getEncoder().getVelocity();
  }

  @AutoLogOutput(key = "outtake/atSpeed")
  private boolean atSpeed() {
    return getError() < 100 && getError() > -100;
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

    outTakeController.setReference(setPoint, ControlType.kVelocity);

    Logger.recordOutput("outTake/Output", outtakeMotor.getAppliedOutput());
    Logger.recordOutput("outTake/speed", outtakeMotor.getEncoder().getVelocity());
    Logger.recordOutput("outTake/setPoint", setPoint);
  }
}
