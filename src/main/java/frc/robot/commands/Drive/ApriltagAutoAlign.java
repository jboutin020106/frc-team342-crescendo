// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;

public class ApriltagAutoAlign extends Command {

  private final SwerveDrive swerve;
  private final SlewRateLimiter xLimiter, yLimiter, rotateLimiter;
  private final PIDController drivePID, strafePID, rotatePID;
  private final double driveOffset, strafeOffset, rotateOffset;
  private final NetworkTable limelightTable;
  private final double[] robotPose;
  private final String limelightName;
  

  /** Creates a new ApriltagAutoAlign. */
  public ApriltagAutoAlign(SwerveDrive swerve, String limelightName) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.limelightName = limelightName;

    this.limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);

    this.swerve = swerve;

    robotPose = limelightTable.getEntry("botpose").getDoubleArray(new double[6]);

    this.xLimiter = new SlewRateLimiter(3);
    this.yLimiter = new SlewRateLimiter(3);
    this.rotateLimiter = new SlewRateLimiter(3);

    this.drivePID = new PIDController(
      AutoAlignConstants.DRIVE_PID_P, 
      AutoAlignConstants.DRIVE_PID_I, 
      AutoAlignConstants.DRIVE_PID_D);

    this.strafePID = new PIDController(
      AutoAlignConstants.STRAFE_PID_P, 
      AutoAlignConstants.STRAFE_PID_I, 
      AutoAlignConstants.STRAFE_PID_D);

    this.rotatePID = new PIDController(
      AutoAlignConstants.ROTATE_PID_P, 
      AutoAlignConstants.ROTATE_PID_I, 
      AutoAlignConstants.ROTATE_PID_D);

      if(limelightName.equals(LimelightConstants.AMP_SIDE_LIMELIGHT_NAME)){

        this.driveOffset = LimelightConstants.AMP_SIDE_APRILTAG_DRIVE_OFFSET;
        this.strafeOffset = LimelightConstants.AMP_SIDE_APRILTAG_STRAFE_OFFSET;
        this.rotateOffset = LimelightConstants.AMP_SIDE_APRILTAG_ROTATE_OFFSET;

      } else if(limelightName.equals(LimelightConstants.SHOOTER_SIDE_LIMELIGHT_NAME)){

        driveOffset = LimelightConstants.SHOOTER_SIDE_APRILTAG_DRIVE_OFFSET;
        strafeOffset = LimelightConstants.SHOOTER_SIDE_APRILTAG_STRAFE_OFFSET;
        rotateOffset = LimelightConstants.SHOOTER_SIDE_APRILTAG_ROTATE_OFFSET;

      } else {

        driveOffset = 0;
        strafeOffset = 0;
        rotateOffset = 0;
        
      }

    

      addRequirements(swerve);

  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      double driveVelocity = 0;
      double strafeVelocity = 0;
      double rotateVelocity = 0;

      if(LimelightHelpers.getTV("Limelight")){

        driveVelocity = drivePID.calculate(LimelightHelpers.getTA(limelightName), driveOffset);
        strafeVelocity = strafePID.calculate(LimelightHelpers.getTX(limelightName), strafeOffset);
        rotateVelocity = rotatePID.calculate(-robotPose[5], rotateOffset);

      } else if (LimelightHelpers.getTV(limelightName) == false) {

        driveVelocity = 0;
        strafeVelocity = 0;
        rotateVelocity = 0.4;

      } else {

        driveVelocity = 0;
        strafeVelocity = 0;
        rotateVelocity = 0;

      }

      //Applies deadband to speed
      driveVelocity = Math.abs(driveVelocity) > AutoAlignConstants.AUTO_ALIGN_DEADBAND ? driveVelocity : 0.0;
      strafeVelocity = Math.abs(strafeVelocity) > AutoAlignConstants.AUTO_ALIGN_DEADBAND ? strafeVelocity : 0.0;
      rotateVelocity = Math.abs(rotateVelocity) > AutoAlignConstants.AUTO_ALIGN_DEADBAND ? driveVelocity : 0.0;

      //Makes the driving smoother
      driveVelocity = xLimiter.calculate(driveVelocity) * 3;
      strafeVelocity = yLimiter.calculate(strafeVelocity) * 3;
      rotateVelocity = rotateLimiter.calculate(rotateVelocity) * 3;

      //Constructs chassis speeds and applies them to robot
      ChassisSpeeds autoAlignSpeeds;

      autoAlignSpeeds = new ChassisSpeeds(driveVelocity, strafeVelocity, rotateVelocity);

      SwerveModuleState[] moduleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(autoAlignSpeeds);

      swerve.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
