package frc.robot.subsystems.elevator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final SparkMax highElevatorMotor = new SparkMax(10, MotorType.kBrushless);
  private final SparkMax lowElevatorMotor = new SparkMax(11, MotorType.kBrushless);

  public Elevator() {
    var highSparkMaxConfig = new SparkMaxConfig();

    highSparkMaxConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40, 40)
        .secondaryCurrentLimit(50)
        .inverted(sparkStickyFault);

    highSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0, 0, 0);

    tryUntilOk(
        highElevatorMotor,
        5,
        () ->
            highElevatorMotor.configure(
                highSparkMaxConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    var lowSparkMaxConfig = new SparkMaxConfig();

    lowSparkMaxConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40, 40).secondaryCurrentLimit(50);

    lowSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0, 0, 0);

    tryUntilOk(
        lowElevatorMotor,
        5,
        () ->
            lowElevatorMotor.configure(
                lowSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }
}
