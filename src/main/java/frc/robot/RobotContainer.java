// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.ClimbCage;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOCanandgyro;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Elevator m_elevator = new Elevator();
  private final Outtake m_outtake = new Outtake();
  private final ClimbCage m_ClimbCage = new ClimbCage();
  // Controller
  private final CommandJoystick rightJoystick = new CommandJoystick(1);
  private final CommandJoystick leftJoystick = new CommandJoystick(0);
  private final CommandXboxController xboxController = new CommandXboxController(2);
  // Sensors
  private final UsbCamera driveCamera;

  private void configureAutoCommands() {
    NamedCommands.registerCommands(
        Map.of(
            "pickUp",
            m_outtake.autoIntake(10),
            "placeL1",
            Commands.sequence(
                m_elevator.levelOne(),
                Commands.waitSeconds(0.125),
                m_outtake.placeCoral(),
                Commands.waitSeconds(0.125),
                m_outtake.pauseOutTake(),
                m_elevator.pickUp()),
            "placeL2",
            Commands.sequence(
                m_elevator.levelTwo(),
                Commands.waitSeconds(0.125),
                m_outtake.placeCoral(),
                Commands.waitSeconds(0.125),
                m_outtake.pauseOutTake(),
                m_elevator.pickUp()),
            "placeL3",
            Commands.sequence(
                m_elevator.levelThree(),
                Commands.waitSeconds(0.125),
                m_outtake.placeCoral(),
                Commands.waitSeconds(0.125),
                m_outtake.pauseOutTake(),
                m_elevator.pickUp()),
            "placeL4",
            Commands.sequence(
                m_elevator.levelFour(),
                Commands.waitSeconds(0.125),
                m_outtake.slowPlaceCoral(),
                Commands.waitSeconds(0.125),
                m_outtake.pauseOutTake(),
                m_elevator.pickUp())));
  }

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOCanandgyro(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Set up auto routines
    configureAutoCommands();
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    driveCamera = CameraServer.startAutomaticCapture();
    driveCamera.setResolution(320, 240);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -leftJoystick.getY(),
            () -> -leftJoystick.getX(),
            () -> -rightJoystick.getX()));

    // Lock to 0° when A button is held
    /*  xboxController
    .a()
    .whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -leftJoystick.getY(),
            () -> -leftJoystick.getX(),
            () -> new Rotation2d())); */

    // Switch to X pattern when X button is pressed
    // leftJoystick.trigger().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when triger is pressed
    rightJoystick
        .trigger()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    xboxController
        .rightTrigger()
        .onTrue(m_ClimbCage.climbCageSlow())
        .onFalse(m_ClimbCage.pauseClimbCage());
    xboxController
        .leftTrigger()
        .onTrue(m_ClimbCage.reverseClimbCageSlow())
        .onFalse(m_ClimbCage.pauseClimbCage());
    xboxController
        .rightBumper()
        .onTrue(m_ClimbCage.climbCageFast())
        .onFalse(m_ClimbCage.pauseClimbCage());

    xboxController.povUp().onTrue(m_elevator.levelFour());
    xboxController.povDown().onTrue(m_elevator.zero());
    xboxController.povRight().onTrue(m_elevator.levelTwo());
    xboxController.povLeft().onTrue(m_elevator.levelThree());
    xboxController.a().onTrue(m_elevator.levelOne());
    xboxController.y().onTrue(m_outtake.slowPlaceCoral()).onFalse(m_outtake.pauseOutTake());
    xboxController.b().onTrue(m_outtake.reverseOutTake()).onFalse(m_outtake.pauseOutTake());
    xboxController.x().onTrue(m_outtake.placeCoral()).onFalse(m_outtake.pauseOutTake());
    m_elevator.setDefaultCommand(
        m_elevator.defaultCommand(() -> MathUtil.applyDeadband(xboxController.getLeftY(), 0.2)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
