package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
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
  private double setPoint = 0;
  private double armPosition = 0;
  private boolean climberReset = false;
  private double resetPosition = 0;
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
        .pidf(.0001, 0.0, 0.0, 1.774691358024691e-4);

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

  public Command climbCage() {
    climberReset = false;
    return runOnce(
        () -> {
          setPoint = 70;
        });
  }

  public Command resetClimber() {
    return runOnce(
        () -> {
          climberReset = true;
          if (armPosition >= resetPosition) {
            setPoint = -50;
            System.out.println("Set Point: " + setPoint);
            System.out.println("Arm Position: " + armPosition);
          } else {
            setPoint = 50;
            System.out.println("Set Point: " + setPoint);
            System.out.println("Arm Position: " + armPosition);
          }
        });
  }

  public Command reverseClimbCage() {
    climberReset = false;
    return runOnce(
        () -> {
          setPoint = -100;
        });
  }

  public Command pauseClimbCage() {
    climberReset = false;
    return runOnce(
        () -> {
          setPoint = 0;
        });
  }

  @Override
  public void periodic() {

    armPosition = shoulderEncoder.getPosition();
    if (armPosition > .50) {
      armPosition = armPosition - 1;
    }
    if (armPosition >= .25) {
      if (setPoint > 0) {
        setPoint = 0;
      }
    }
    if (armPosition <= -.25) {
      if (setPoint < 0) {
        setPoint = 0;
      }
    }
    System.out.println("Set Point: " + setPoint);
    System.out.println("Arm Position: " + armPosition);
    if (climberReset == true) {
      if ((armPosition < resetPosition + 0.05) && (armPosition > resetPosition - 0.05)) {
        System.out.println("Set Point: " + setPoint);
        setPoint = 0;
        System.out.println("Reset turns back to false");
        climberReset = false;
      }
    }

    ClimbCageController.setReference(setPoint, ControlType.kVelocity);

    Logger.recordOutput("ClimbCage/Output", ClimbCageMotor.getAppliedOutput());
    Logger.recordOutput("ClimbCage/speed", ClimbCageMotor.getEncoder().getVelocity());
    Logger.recordOutput("ClimbCage/setPoint", setPoint);
    Logger.recordOutput("ClimbCage/reset", climberReset);
  }
}
