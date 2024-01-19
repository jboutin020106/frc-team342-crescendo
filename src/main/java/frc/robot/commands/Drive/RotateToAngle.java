// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

public class RotateToAngle extends Command {
 
  private final SwerveDrive swerve;
  private final ProfiledPIDController rotateController;

  private Rotation2d startAngle;
  private Rotation2d endAngle;

  private final Rotation2d angle;

  private final TrapezoidProfile.Constraints((DriveConstants.MAX_ROTATE_SPEED / 2), MAX_DRIVE_SPEED)

  /** Creates a new RotateToAngle. */
  public RotateToAngle(Rotation2d angle, SwerveDrive swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    addRequirements(this.swerve);

    this.angle = angle;

    rotateController = new ProfiledPIDController(
      DriveConstants.ROTATION_P, 
      DriveConstants.ROTATION_I, 
      DriveConstants.ROTATION_D,
      );

      rotateController.setTolerance(DriveConstants.ROTATION_TOLERANCE);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    startAngle = new Rotation2d(swerve.getHeading());

    endAngle = startAngle.plus(angle);

    rotateController.reset();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Rotation2d currentAngle = new Rotation2d(swerve.getHeading());

    double rotationVelocity = rotateController.calculate(currentAngle.getRadians(), endAngle.getRadians());

    ChassisSpeeds radialSpeeds = new ChassisSpeeds(0, 0, -rotationVelocity);

    HolonomicDriveController
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
