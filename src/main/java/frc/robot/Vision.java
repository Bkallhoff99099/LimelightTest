// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private String m_name;
  private SwerveDrivePoseEstimator m_PoseEstimator;
  private AHRS m_gyro;
  private boolean doRejectUpdate;
  public Vision(String limelightName,  AHRS gyro, SwerveDrivePoseEstimator poseEstimator) {
    m_name  = limelightName;
    m_gyro = gyro;
    m_PoseEstimator = poseEstimator;
    
  }

  public Pose2d getPose(){
    return LimelightHelpers.getBotPose2d_wpiBlue(m_name);
  }

  public double aimWithVision(){
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX(m_name) * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= DriveConstants.kMaxAngularSpeed;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  @Override
  public void periodic() {
    doRejectUpdate = false;
    LimelightHelpers.SetRobotOrientation(m_name, m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate megaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_name);
    if(Math.abs(m_gyro.getRate()) > 360){
      doRejectUpdate = true;
    }
    if(megaTag2.tagCount == 0){
      doRejectUpdate = true;
    }
    if(!doRejectUpdate){
      m_PoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7, 9999999));
      m_PoseEstimator.addVisionMeasurement(megaTag2.pose, megaTag2.timestampSeconds);
      
    }
    
    

  }
}
