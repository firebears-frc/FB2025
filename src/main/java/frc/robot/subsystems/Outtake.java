package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import static jdk.internal.vm.ScopedValueContainer.run;

public class Outtake extends SubsystemBase {
    private CANSparkMax outtakeMotor;
    private SparkPIDController pid;
    private DigitalInput sensor;
    private double setPoint = 0;
    @AutoLogOutput(key="outTake/hasNote")
    private boolean hasNote = false;

    public Outtake() {

        SparkMax outTakeMotor =
        new SparkMax(
            MotorType.kBrushless);
    driveEncoder = driveSpark.getEncoder();
    turnEncoder = turnSpark.getAbsoluteEncoder();
    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();
            // Configure turn motor
    var outTakeConfig = new SparkMaxConfig();
    outTakeConfig
        .inverted(turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    outTakeConfig
        .absoluteEncoder
        .inverted(turnEncoderInverted)
        .positionConversionFactor(turnEncoderPositionFactor)
        .velocityConversionFactor(turnEncoderVelocityFactor)
        .averageDepth(2);
    outTakeConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .pidf(turnKp, 0.0, turnKd, 0.0);
    outTakeConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                outTakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @AutoLogOutput(key = "outtake/error")
    private double getError(){
        return setPoint-outtakeMotor.getEncoder().getVelocity();
    }

    @AutoLogOutput(key = "downBeat/atSpeed")
    private boolean atSpeed(){
        if((getError()<100)&&(getError()>-100)){
            return true;
        }
        return false;
    }

    public Command intakeNote() {
        return runOnce(() -> {
            setPoint = 7000;
        });
    }

    public Command shootNote(){
        return runOnce(() -> {
            setPoint = 8000;
        });
    }

    public Command dischargeNote() {
        return runOnce(() -> {
            setPoint = -7000;
        });
    }

    public Command pauseDownBeat() {
        return runOnce(() -> {
            setPoint = 0;
        });
    }

    public Command autoIntake(double timeOut) {
        return Commands.sequence(runOnce(() -> setPoint = 7000),
        run(()-> {}).until(()->hasNote).withTimeout(timeOut));
    }

    @Override
    public void periodic() {
        if(beamBreak() && !hasNote){
            setPoint = 0;
            hasNote = true;
        } else if(!beamBreak()){
            hasNote = false;
        }
        pid.setReference(setPoint, ControlType.kVelocity);
        
        Logger.recordOutput("outTake/Output", outtakeMotor.getAppliedOutput());
        Logger.recordOutput("outTake/speed", outtakeMotor.getEncoder().getVelocity());
        Logger.recordOutput("outTake/setPoint", setPoint);
    }
}