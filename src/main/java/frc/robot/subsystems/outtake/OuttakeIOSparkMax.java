package frc.robot.subsystems.outtake;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.util.Units;
import frc.robot.util.SparkUtil;

public class OuttakeIOSparkMax implements OuttakeIO {
  private final SparkMax motor = new SparkMax(12, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final SparkClosedLoopController closedLoopController = motor.getClosedLoopController();

  public OuttakeIOSparkMax() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12).smartCurrentLimit(20, 10).secondaryCurrentLimit(25.0);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    inputs.velocityRadiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.appliedCurrent = motor.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    closedLoopController.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec),
        ControlType.kVelocity);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {

    SparkUtil.tryUntilOk(motor, 5, () -> motor.configure(null, null, null)));
  }
}
