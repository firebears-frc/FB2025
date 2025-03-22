package frc.robot.subsystems.outtake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class OuttakeIOSim implements OuttakeIO {
  private final DCMotor motor = DCMotor.getNEO(1);
  private final LinearSystem<N1, N1, N1> plant =
      LinearSystemId.createFlywheelSystem(motor, 0.004, Outtake.Constants.GEAR_RATIO);
  private final FlywheelSim sim = new FlywheelSim(plant, motor);
  private final PIDController pid = new PIDController(0.0, 0.0, 0.0);
  private final LoggedNetworkBoolean beamBrakeSim =
      new LoggedNetworkBoolean("SmartDashboard/Intake/Beam Brake Simulation", false);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(pid.calculate(sim.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
      sim.setInputVoltage(appliedVolts);
    }

    sim.update(0.02);

    inputs.positionRadians += sim.getAngularVelocityRadPerSec() * 0.02;
    inputs.velocityRadiansPerSecond = sim.getAngularAccelerationRadPerSecSq();
    inputs.appliedVolts = appliedVolts;
    inputs.appliedCurrent = sim.getCurrentDrawAmps();
    inputs.beamBrake = beamBrakeSim.get();
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    closedLoop = true;
    pid.setSetpoint(velocityRadPerSec);
    this.ffVolts = ffVolts;
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
