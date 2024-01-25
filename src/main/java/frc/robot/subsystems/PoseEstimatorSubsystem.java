// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.channels.OverlappingFileLockException;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;

public class PoseEstimatorSubsystem extends SubsystemBase {

  private static final SwerveDrive swerve;
  
  //Standard Deviation instantiations
  private static final Matrix stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
  private static final Matrix visionStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);

  //Supplier instantations
  private final Supplier<Rotation2d> rotationSupplier;
  private final Supplier<SwerveModulePosition[]> modulePositionSupplier;
  private final SwerveDrivePoseEstimator poseEstimator;

  //Field 2D instantiation
  private final Field2d field2d = new Field2d();

  private OriginPosition originPosition = kBlueAllianceWallRightSide;
  private boolean sawTag = false;

  swerve = new SwerveDrive();


  /** Creates a new PoseEstimatorSubsystem. */
  public PoseEstimatorSubsystem(SwerveDrive swerve) {

          this.rotationSupplier = rotationSupplier;
          this.modulePositionSupplier = modulePositionSupplier;

          this.swerve = swerve;

          poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.KINEMATICS,
            swerve.getRotation2d(), 
            swerve.getModulePositions(),
            new Pose2d() 
            );
  }

  public void addDashboardWidgets(ShuffleboardTab tab){
    tab.add("Field", field2d).withPosition(0, 0).withSize(6, 4);
    tab.addString("Pose", this::getFormattedPose)
  }

  private String getFormattedPose(){
    var pose = getCurrentPose();

    return String.format("(%.3f, %.3f) %.2f degrees", 
    pose.getX(),
    pose.getY(),
    pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose(){

    return poseEstimator.getEstimatedPosition();

  }

  public void setCurrentPose(Pose2d newPose){
    poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), newPose);
  }

  public void resetFieldPosition(){
    setCurrentPose(new Pose2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
