// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.derive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.VisionMoveTo;



public class RobotContainer {
  final CommandXboxController m_driveController = new CommandXboxController(0);
  final DriveSubsystem m_drive = new DriveSubsystem();
  public RobotContainer() {
    configureBindings();
    LimelightHelpers.setLEDMode_ForceOn("");
  }

  private void configureBindings() {
    m_drive.setDefaultCommand(new RunCommand(()->
    m_drive.drive(
        -MathUtil.applyDeadband(m_driveController.getLeftY(), 0.2),
        -MathUtil.applyDeadband(m_driveController.getLeftX(), 0.2),
        -MathUtil.applyDeadband(m_driveController.getRightX(), 0.2),
        true), m_drive
    ));
    
    
    m_driveController.button(8).onTrue(new InstantCommand(()-> m_drive.zeroHeading()));

    //new Trigger(m_driveController.button(1) && m_driveController.button(2)).onTrue(new VisionMoveTo());

    

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
