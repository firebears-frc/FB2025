package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private SparkMax outtakeMotor = new SparkMax(12, MotorType.kBrushless);
  private final SparkClosedLoopController outTakeController;
  private SparkLimitSwitch beamBreak = outtakeMotor.getForwardLimitSwitch();
  private double setPoint = 0;
  private static final int outtakeCurrentLimit = 30;

  @AutoLogOutput(key = "outtake/hasCoral")
  private boolean hasCoral = false;

  public Outtake() {

    // Configure turn motor
    outTakeController = outtakeMotor.getClosedLoopController();
    var outTakeConfig = new SparkMaxConfig();
    outTakeConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(outtakeCurrentLimit)
        .secondaryCurrentLimit(50)
        .voltageCompensation(12.0);
    outTakeConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(.00007, 0.0, 0.0, 1.774691358024691e-4);
    // ff: 1.774691358024691e-4
    outTakeConfig.limitSwitch.forwardLimitSwitchEnabled(false);

    SparkUtil.tryUntilOk(
        outtakeMotor,
        5,
        () ->
            outtakeMotor.configure(
                outTakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @AutoLogOutput(key = "outtake/beamBreak")
  private boolean beamBreak() {
    return beamBreak.isPressed();
  }

  @AutoLogOutput(key = "outtake/error")
  private double getError() {
    return setPoint - outtakeMotor.getEncoder().getVelocity();
  }

  @AutoLogOutput(key = "outtake/atSpeed")
  private boolean atSpeed() {
    return getError() < 100 && getError() > -100;
  }

  public Command reverseOutTake() {
    return runOnce(
        () -> {
          setPoint = 1000;
        });
  }

  public Command slowPlaceCoral() {
    return runOnce(
        () -> {
          setPoint = -1000;
        });
  }

  public Command placeCoral() {
    return runOnce(
        () -> {
          setPoint = -5000;
        });
  }

  public Command pauseOutTake() {
    return runOnce(
        () -> {
          setPoint = 0;
        });
  }

  public Command autoIntake(double timeOut) {
    return Commands.sequence(
        runOnce(() -> setPoint = 7000), run(() -> {}).until(() -> hasCoral).withTimeout(timeOut));
  }

  @Override
  public void periodic() {
    if (beamBreak() && !hasCoral) {
      setPoint = 0;
      hasCoral = true;
    } else if (!beamBreak()) {
      hasCoral = false;
    }

    outTakeController.setReference(setPoint, ControlType.kVelocity);

    Logger.recordOutput("outTake/Output", outtakeMotor.getAppliedOutput());
    Logger.recordOutput("outTake/speed", outtakeMotor.getEncoder().getVelocity());
    Logger.recordOutput("outTake/setPoint", setPoint);
  }
}
