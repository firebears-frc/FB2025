package frc.robot.subsystems.climber;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.SparkUtil;

public class ClimberIOSparkMax implements ClimberIO {
  private final SparkMax motor = new SparkMax(13, MotorType.kBrushless);
  private final SparkAbsoluteEncoder encoder = motor.getAbsoluteEncoder();
  private final SparkClosedLoopController controller = motor.getClosedLoopController();
  private final SparkMaxConfig config = new SparkMaxConfig();

  public ClimberIOSparkMax() {
    config
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .smartCurrentLimit(50, 30)
        .secondaryCurrentLimit(60.0);
    SparkUtil.configure(motor, config);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.position = Rotation2d.fromRotations(encoder.getPosition());
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
  }

  @Override
  public void setPosition(Rotation2d angle, double ffVolts) {
    controller.setReference(
        angle.getRotations(),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setVoltage(double volts) {
    motor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    config.closedLoop.pidf(kP, kI, kD, 0.0, ClosedLoopSlot.kSlot0);
    SparkUtil.configure(motor, config);
  }
}
