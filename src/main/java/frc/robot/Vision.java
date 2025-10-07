// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private String name;
  private SwerveDrivePoseEstimator m_PoseEstimator;
  private AHRS m_gyro;
  private boolean doRejectUpdate;
  public Vision(String limelightName,  AHRS gyro, SwerveDrivePoseEstimator poseEstimator) {
    name  = limelightName;
    m_gyro = gyro;
    m_PoseEstimator = poseEstimator;
    
  }

  @Override
  public void periodic() {
    doRejectUpdate = false;
    LimelightHelpers.SetRobotOrientation(name, m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate megaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
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
    
    

    // This method will be called once per scheduler run
   // m_PoseEstimator.addVisionMeasurement( );
  }
}
