package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ClimberCommands {
  private static final Map<String, LoggedNetworkNumber> angles = new HashMap<>();

  public static enum Angle {
    STOW(90.0),
    GRAB(0.0),
    CLIMB(120.0);

    public final double defaultAngleDegrees;

    private Angle(double defaultAngleDegrees) {
      this.defaultAngleDegrees = defaultAngleDegrees;
    }
  }

  public ClimberCommands() {
    for (Angle angle : Angle.values()) {
      angles.put(
          angle.name(),
          new LoggedNetworkNumber(
              "SmartDashboard/Climber Angles/" + angle.name(), angle.defaultAngleDegrees));
    }
  }

  /** Returns a command to run the climber. */
  public static Command angle(Climber climber, Angle angle) {
    return Commands.sequence(
        Commands.runOnce(
            () -> climber.set(Rotation2d.fromDegrees(angles.get(angle.name()).get())), climber),
        Commands.waitSeconds(0.25),
        Commands.run(() -> {}, climber).until(climber::onTarget));
  }
}
