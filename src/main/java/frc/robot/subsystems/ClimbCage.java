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
  private SparkMax ClimbCageMotor = new SparkMax(13, MotorType.kBrushless);
  private final SparkClosedLoopController ClimbCageController;
  // Adjust to desired value  -- change
  private double slowSpeed = 600;
  private double fastSpeed = 6000;
  private double setPointSpeed = 250;
  private double resetPosition = .0;
  private double outPosition = .25;
  private double inPosition = -.25;
  // Might need to change upon install
  private double setPointDirection = -1;
  // Won't need to change these
  @AutoLogOutput(key = "ClimbCage/Setpoint")
  private double setPoint = 0;

  @AutoLogOutput(key = "ClimbCage/Position")
  private double armPosition = 0;

  @AutoLogOutput(key = "ClimbCage/out")
  private boolean climberOut = false;

  @AutoLogOutput(key = "ClimbCage/in")
  private boolean climberIn = false;

  @AutoLogOutput(key = "ClimbCage/reset")
  private boolean climberReset = false;

  private static AbsoluteEncoder shoulderEncoder;

  private static final int ClimbCageCurrentLimit = 50;

  public ClimbCage() {

    // Configure turn motor
    shoulderEncoder = ClimbCageMotor.getAbsoluteEncoder();
    ClimbCageController = ClimbCageMotor.getClosedLoopController();
    var ClimbCageConfig = new SparkMaxConfig();
    ClimbCageConfig.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ClimbCageCurrentLimit)
        .secondaryCurrentLimit(60)
        .voltageCompensation(12.0);
    ClimbCageConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(.001, 0.0, 0.0, 1.774691358024691e-4);

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

  // MANUAL COMMANDS
  public Command climbCageSlow() {
    return runOnce(
        () -> {
          climberReset = false;
          climberIn = false;
          climberOut = false;
          setPoint = slowSpeed * setPointDirection;
        });
  }

  public Command reverseClimbCageSlow() {
    return runOnce(
        () -> {
          climberReset = false;
          climberIn = false;
          climberOut = false;
          setPoint = -slowSpeed * setPointDirection;
        });
  }

  public Command climbCageFast() {
    return runOnce(
        () -> {
          climberReset = false;
          climberIn = false;
          climberOut = false;
          setPoint = fastSpeed * setPointDirection;
        });
  }

  public Command reverseClimbCageFast() {
    return runOnce(
        () -> {
          climberReset = false;
          climberIn = false;
          climberOut = false;
          setPoint = -fastSpeed * setPointDirection;
        });
  }

  public Command pauseClimbCage() {
    return runOnce(
        () -> {
          climberReset = false;
          climberIn = false;
          climberOut = false;
          setPoint = 0;
        });
  }

  // SETPOINT COMMANDS
  public Command resetClimber() {
    return runOnce(
        () -> {
          climberReset = true;
          climberOut = false;
          climberIn = false;
          if (armPosition >= resetPosition) {
            setPoint = -setPointSpeed * setPointDirection;
          } else {
            setPoint = setPointSpeed * setPointDirection;
          }
        });
  }

  public Command outClimber() {
    return runOnce(
        () -> {
          climberOut = true;
          climberIn = false;
          climberReset = false;
          if (armPosition >= outPosition) {
            setPoint = -setPointSpeed * setPointDirection;
          } else {
            setPoint = setPointSpeed * setPointDirection;
          }
        });
  }

  public Command inClimber() {
    return runOnce(
        () -> {
          climberIn = true;
          climberOut = false;
          climberReset = false;
          if (armPosition >= inPosition) {
            setPoint = -setPointSpeed * setPointDirection;
          } else {
            setPoint = setPointSpeed * setPointDirection;
          }
        });
  }

  @Override
  public void periodic() {
    if (shoulderEncoder.getPosition() < .50) {
      armPosition = shoulderEncoder.getPosition();
    }
    if (shoulderEncoder.getPosition() > .50) {
      armPosition = shoulderEncoder.getPosition() - 1;
    }

    // setPoint based inequalities may need to be flipped upon install -- change
    if (armPosition >= .25) {
      if (setPoint < 0) {
        setPoint = 0;
      }
    }
    if (armPosition <= -.164) {
      if (setPoint > 0) {
        setPoint = 0;
      }
    }
    if (climberReset == true) {
      if ((armPosition < resetPosition + 0.01) && (armPosition > resetPosition - 0.01)) {
        setPoint = 0;
        System.out.println("climberReset turns back to false");
        climberReset = false;
      }
    }
    if (climberIn == true) {
      if ((armPosition < inPosition + 0.01) && (armPosition > inPosition - 0.01)) {
        setPoint = 0;
        System.out.println("climberIn turns back to false");
        climberIn = false;
      }
    }
    if (climberOut == true) {
      if ((armPosition < outPosition + 0.01) && (armPosition > outPosition - 0.01)) {
        setPoint = 0;
        System.out.println("climberOut turns back to false");
        climberOut = false;
      }
    }

    ClimbCageController.setReference(setPoint, ControlType.kVelocity);

    Logger.recordOutput("ClimbCage/Output", ClimbCageMotor.getAppliedOutput());
    Logger.recordOutput("ClimbCage/speed", ClimbCageMotor.getEncoder().getVelocity());
    Logger.recordOutput("ClimbCage/setPoint", setPoint);
    Logger.recordOutput("ClimbCage/reset", climberReset);
  }
}
