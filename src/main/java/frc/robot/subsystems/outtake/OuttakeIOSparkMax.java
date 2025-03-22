package frc.robot.subsystems.outtake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SparkUtil;

public class OuttakeIOSparkMax implements OuttakeIO {
  private final SparkMax motor = new SparkMax(12, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final SparkClosedLoopController closedLoopController = motor.getClosedLoopController();
  private final SparkLimitSwitch beamBrake = motor.getForwardLimitSwitch();
  private final SparkMaxConfig config = new SparkMaxConfig();

  public OuttakeIOSparkMax() {
    config.voltageCompensation(12).smartCurrentLimit(50, 30).secondaryCurrentLimit(60.0);
    SparkUtil.configure(motor, config);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    inputs.positionRadians =
        Units.rotationsToRadians(encoder.getPosition() / Outtake.Constants.GEAR_RATIO);
    inputs.velocityRadiansPerSecond =
        Units.rotationsPerMinuteToRadiansPerSecond(
            encoder.getVelocity() / Outtake.Constants.GEAR_RATIO);
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.appliedCurrent = motor.getOutputCurrent();
    inputs.beamBrake = beamBrake.isPressed();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    closedLoopController.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec)
            * Outtake.Constants.GEAR_RATIO,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    config.closedLoop.pidf(kP, kI, kD, 0);
    SparkUtil.configure(motor, config);
  }
}
