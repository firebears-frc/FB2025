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

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimbCage;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final ClimbCage m_ClimbCage = new ClimbCage();
  // private final Climber m_climber = new Climber();
  // Controller

  private final CommandXboxController xboxController = new CommandXboxController(0);

  // Dashboard inputs

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Set up auto routines

    // Set up SysId routines

    // Configure the button bindings
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

    // Reset gyro to 0° when triger is pressed
    // xboxController.x().onTrue(m_ClimbCage.ClimbCageCoral()).onFalse(m_ClimbCage.pauseClimbCage());
    // xboxController.y().onTrue(m_ClimbCage.reverseClimbCage()).onFalse(m_ClimbCage.pauseClimbCage());
    // xboxController.b().onTrue(m_ClimbCage.pauseClimbCage());
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
    xboxController
        .leftBumper()
        .onTrue(m_ClimbCage.reverseClimbCageFast())
        .onFalse(m_ClimbCage.pauseClimbCage());
    xboxController.x().onTrue(m_ClimbCage.resetClimber());
    xboxController.a().onTrue(m_ClimbCage.inClimber());
    xboxController.b().onTrue(m_ClimbCage.outClimber());
    xboxController.y().onTrue(m_ClimbCage.pauseClimbCage());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
