// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.VisionMoveTo;
import frc.robot.Constants.CameraNames;
import frc.robot.Constants.LocationConstants;
import frc.robot.LimelightHelpers.PoseEstimate;



public class RobotContainer {
  final XboxController m_driveController = new XboxController(0);
  final DriveSubsystem m_drive = new DriveSubsystem();
  final Vision limelight1 = new Vision( CameraNames.kFrontCamera, m_drive.getGyro(), m_drive.getPoseEstimator());

  public RobotContainer() {
    ShuffleboardTab display = Shuffleboard.getTab("main tab");
    configureBindings();
    LimelightHelpers.setLEDMode_ForceOn(CameraNames.kFrontCamera);


    display.addDouble("DriveTrain X pose",()-> m_drive.getPose().getX());
    display.addDouble("Drivetrain Y pose", ()-> m_drive.getPose().getY());

    display.addDouble("Camera X pose", ()-> limelight1.getPose().getX());
    display.addDouble("Camera Y pose", ()-> limelight1.getPose().getY());

    display.addDouble("Camera TX", ()-> LimelightHelpers.getTX(CameraNames.kFrontCamera));
    display.addDouble("Camera TA", ()-> LimelightHelpers.getTA(CameraNames.kFrontCamera));
  }

  private void configureBindings() {
    m_drive.setDefaultCommand(new RunCommand(()->
    m_drive.drive(
        -MathUtil.applyDeadband(m_driveController.getLeftY(), 0.2),
        -MathUtil.applyDeadband(m_driveController.getLeftX(), 0.2),
        -MathUtil.applyDeadband(m_driveController.getRightX(), 0.2),
        true), m_drive
    ));
    
    
    
    new Trigger(()-> m_driveController.getLeftBumperButton() && m_driveController.getAButton()).onTrue(new VisionMoveTo());
    new Trigger(()-> m_driveController.getRightBumperButton()).whileTrue( new RunCommand(()->
      m_drive.drive(
        -MathUtil.applyDeadband(m_driveController.getLeftY(), 0.2),
        -MathUtil.applyDeadband(m_driveController.getLeftX(), 0.2),
        limelight1.aimWithVision(),
        true), m_drive
    ));

    

  }

  

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

   
    

}
