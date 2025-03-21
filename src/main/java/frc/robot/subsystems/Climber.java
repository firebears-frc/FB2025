package frc.robot.subsystems;

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

public class Climber extends SubsystemBase {
  private SparkMax climberMotor = new SparkMax(8, MotorType.kBrushless);
  private final SparkClosedLoopController climberController;
  // private RelativeEncoder climberEncoder;
  private double setPoint = 0;
  private static final int climberCurrentLimit = 30;

  public Climber() {

    // Configure turn motor
    climberController = climberMotor.getClosedLoopController();
    var climberConfig = new SparkMaxConfig();
    climberConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(climberCurrentLimit)
        .secondaryCurrentLimit(50)
        .voltageCompensation(12.0);
    climberConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(0.0, 0.0, 0.0, 0.00001);

    tryUntilOk(
        climberMotor,
        5,
        () ->
            climberMotor.configure(
                climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
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

  @AutoLogOutput(key = "climber/error")
  private double getError() {
    return setPoint - climberMotor.getEncoder().getVelocity();
  }

  @AutoLogOutput(key = "climber/atSpeed")
  private boolean atSpeed() {
    return getError() < 100 && getError() > -100;
  }

  public Command climberOut() {
    return runOnce(
        () -> {
          setPoint = 100;
        });
  }

  public Command climberIn() {
    return runOnce(
        () -> {
          setPoint = -100;
        });
  }

  public Command pauseClimber() {
    return runOnce(
        () -> {
          setPoint = 0;
        });
  }

  @Override
  public void periodic() {

    climberController.setReference(setPoint, ControlType.kVelocity);

    Logger.recordOutput("climber/Output", climberMotor.getAppliedOutput());
    Logger.recordOutput("climber/speed", climberMotor.getEncoder().getVelocity());
    Logger.recordOutput("climber/setPoint", setPoint);
  }
}
