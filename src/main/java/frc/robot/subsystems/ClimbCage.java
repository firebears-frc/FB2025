package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
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

public class ClimbCage extends SubsystemBase {
  private SparkMax ClimbCageMotor = new SparkMax(8, MotorType.kBrushless);
  private final SparkClosedLoopController ClimbCageController;
  private RelativeEncoder ClimbCageEncoder;
  private double setPoint = 0;
  private double armPosition = 0;
  private boolean goUpright = false;
  private static AbsoluteEncoder shoulderEncoder;

  private static final int ClimbCageCurrentLimit = 30;

  public ClimbCage() {

    // Configure turn motor
    shoulderEncoder = ClimbCageMotor.getAbsoluteEncoder();
    ClimbCageController = ClimbCageMotor.getClosedLoopController();
    var ClimbCageConfig = new SparkMaxConfig();
    ClimbCageConfig.idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ClimbCageCurrentLimit)
        .secondaryCurrentLimit(50)
        .voltageCompensation(12.0);
    ClimbCageConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(.00007, 0.0, 0.0, 1.774691358024691e-4);
    // ff: 1.774691358024691e-4

    tryUntilOk(
        ClimbCageMotor,
        5,
        () ->
            ClimbCageMotor.configure(
                ClimbCageConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
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

  @AutoLogOutput(key = "ClimbCage/error")
  private double getError() {
    return setPoint - ClimbCageMotor.getEncoder().getVelocity();
  }

  @AutoLogOutput(key = "ClimbCage/atSpeed")
  private boolean atSpeed() {
    return getError() < 100 && getError() > -100;
  }

  public Command ClimbCageCoral() {
    return runOnce(
        () -> {
          setPoint = 70;
        });
  }

  public Command resetClimber() {
    if (armPosition > 0 && armPosition <= 50) {
      return runOnce(
          () -> {
            setPoint = -70;
            goUpright = true;
          });
    } else if (armPosition > 50 && armPosition != 0) {
      return runOnce(
          () -> {
            setPoint = 70;
            goUpright = true;
          });
    } else {
      return runOnce(
          () -> {
            setPoint = 0;
          });
    }
  }

  public Command reverseClimbCage() {
    return runOnce(
        () -> {
          setPoint = -100;
        });
  }

  public Command pauseClimbCage() {
    return runOnce(
        () -> {
          setPoint = 0;
        });
  }

  @Override
  public void periodic() {

    armPosition = shoulderEncoder.getPosition();
    if (armPosition >= .25 && armPosition <= .50) {
      if (setPoint > 0) {
        setPoint = 0;
      }
    }
    if (armPosition <= .75 && armPosition > .50) {
      if (setPoint < 0) {
        setPoint = 0;
      }
    }
    if (goUpright == true) {
      if (armPosition > .99 || armPosition < .01) {
        setPoint = 0;
        goUpright = false;
      }
    }
    ClimbCageController.setReference(setPoint, ControlType.kVelocity);

    Logger.recordOutput("ClimbCage/Output", ClimbCageMotor.getAppliedOutput());
    Logger.recordOutput("ClimbCage/speed", ClimbCageMotor.getEncoder().getVelocity());
    Logger.recordOutput("ClimbCage/setPoint", setPoint);
  }
}
