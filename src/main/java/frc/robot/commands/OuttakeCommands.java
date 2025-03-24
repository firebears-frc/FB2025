package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.outtake.Outtake;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class OuttakeCommands {
  private static final Map<String, LoggedNetworkNumber> speeds = new HashMap<>();

  public static enum Speed {
    EJECT(775),
    OUT(550),
    IN(250),
    REVERSE(-250);

    public final double defaultVelocityRPM;

    private Speed(double defaultVelocityRPM) {
      this.defaultVelocityRPM = defaultVelocityRPM;
    }
  }

  public OuttakeCommands() {
    for (Speed speed : Speed.values()) {
      speeds.put(
          speed.name(),
          new LoggedNetworkNumber(
              "SmartDashboard/Outtake Speeds/" + speed.name(), speed.defaultVelocityRPM));
    }
  }

  /** Returns a command to stop the outtake. */
  public static Command stop(Outtake outtake) {
    return Commands.runOnce(outtake::stop, outtake);
  }

  /** Returns a command to run the outtake. */
  public static Command speed(Outtake outtake, Speed speed) {
    return Commands.sequence(
        Commands.runOnce(() -> outtake.run(speeds.get(speed.name()).get()), outtake),
        Commands.waitSeconds(0.25),
        Commands.run(() -> {}, outtake).until(outtake::onTarget));
  }
}
