package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

public class Elevator extends SubsystemBase {
    private CANSparkMax elevator1;
    private CANSparkMax elevator2;
    private SparkPIDController pid;
    
    private double setPoint = 0;

    public Climber() {
        elevator1 = new CANSparkMax(9, MotorType.kBrushless);
        elevator1.setSmartCurrentLimit(10, 10);
        elevator1.setSecondaryCurrentLimit(20);
        elevator1.restoreFactoryDefaults();
        elevator1.setInverted(true);
        elevator1.setIdleMode(IdleMode.kBrake);
        pid = elevator1.getPIDController();
        elevator1.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        elevator1.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
        elevator1.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

        pid.setP(0.00001);
        pid.setI(0.0);
        pid.setD(0.0);
        pid.setFF(0.000115);
        pid.setIZone(100);
        elevator1.burnFlash();

        elevator2 = new CANSparkMax(10, MotorType.kBrushless);
        elevator2.setSmartCurrentLimit(10, 10);
        elevator2.setSecondaryCurrentLimit(20);
        elevator2.restoreFactoryDefaults();
        elevator2.setInverted(true);
        elevator2.setIdleMode(IdleMode.kBrake);
        pid = elevator2.getPIDController();
        elevator2.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        elevator2.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
        elevator2.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

        pid.setP(0.00001);
        pid.setI(0.0);
        pid.setD(0.0);
        pid.setFF(0.000115);
        pid.setIZone(100);
        elevator2.burnFlash();
    }

    private final static class Constants {
        private static final double stop = 0.00;
        private static final double go = 1000;
    }

    private Command speedCommand(double speed) {
        return runOnce(() -> setPoint = speed);
    }

    public Command stopMotor() {
        return speedCommand(Constants.stop);
    }

    public Command startMotor() {
        return speedCommand(Constants.go);
    }


    @Override
    public void periodic() {
        pid.setReference(setPoint, ControlType.kVelocity);
    }
}