package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ElevatorCommands {
  private static final Map<String, LoggedNetworkNumber> postiions = new HashMap<>();

  public static enum Position {
    L4(78.9),
    L3(50.0),
    L2(32.0),
    L1(23.6),
    INTAKE(0.0);

    public final double defaultPositionRotations;

    private Position(double defaultPositionRotations) {
      this.defaultPositionRotations = defaultPositionRotations;
    }
  }

  public ElevatorCommands() {
    for (Position position : Position.values()) {
      postiions.put(
          position.name(),
          new LoggedNetworkNumber(
              "SmartDashboard/Elevator Positions/" + position.name(),
              position.defaultPositionRotations));
    }
  }

  /** Returns a command to run the elevator. */
  public static Command position(Elevator elevator, Position position) {
    return Commands.sequence(
        Commands.runOnce(() -> elevator.set(postiions.get(position.name()).get()), elevator),
        Commands.waitSeconds(0.25),
        Commands.run(() -> {}, elevator).until(elevator::onTarget));
  }
}
