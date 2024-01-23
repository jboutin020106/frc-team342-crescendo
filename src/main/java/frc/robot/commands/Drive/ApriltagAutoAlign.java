// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;

public class ApriltagAutoAlign extends Command {

  private final SwerveDrive swerve;
  private final SlewRateLimiter xLimiter, yLimiter, rotateLimiter;
  private final PIDController drivePID, strafePID, rotatePID;
  private final double driveOffset, strafeOffset, rotateOffset;


  /** Creates a new ApriltagAutoAlign. */
  public ApriltagAutoAlign(SwerveDrive swerve) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.swerve = swerve;

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

      this.driveOffset = LimelightConstants.APRILTAG_DRIVE_OFFSET;
      this.strafeOffset = LimelightConstants.APRILTAG_STRAFE_OFFSET;
      this.rotateOffset = LimelightConstants.APRILTAG_ROTATE_OFFSET;

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

        driveVelocity = drivePID.calculate(LimelightHelpers.getTA("Limelight"), driveOffset);
        strafeVelocity = strafePID.calculate(LimelightHelpers.getTX("Limelight"), strafeOffset);
        rotateVelocity = rotatePID.calculate(LimelightHelpers.get)

      }
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
