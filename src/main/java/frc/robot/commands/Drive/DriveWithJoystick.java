// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.SwerveDrive;

public class DriveWithJoystick extends Command {

  private SwerveDrive swerve;
  private XboxController joy;
  private AHRS gyro;

  private Supplier<Double> rotateSpeed;
  private boolean fieldOriented;
  private boolean driveWithTargeting;

  private SlewRateLimiter xLimiter;
  private SlewRateLimiter yLimiter;
  private SlewRateLimiter rotateLimiter;

  private ChassisSpeeds chassisSpeeds;
  private SwerveModuleState[] moduleStates;
  private SwerveDriveKinematics swerveKinematics;

  /** Creates a new DriveWithJoystick. */
  public DriveWithJoystick(SwerveDrive swerve, XboxController joy, boolean fieldOriented, boolean driveWithTargeting) {

    this.swerve = swerve;
    this.joy = joy;
    this.fieldOriented = fieldOriented;
    this.driveWithTargeting = driveWithTargeting;

    xLimiter = new SlewRateLimiter(3);
    yLimiter = new SlewRateLimiter(3);
    rotateLimiter = new SlewRateLimiter(3);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = joy.getLeftY();
    double ySpeed = joy.getLeftX();
    double rotateSpeed = joy.getRawAxis(4);

    xSpeed = MathUtil.applyDeadband(xSpeed, 0.15);
    ySpeed = MathUtil.applyDeadband(ySpeed, 0.15);
    rotateSpeed = MathUtil.applyDeadband(rotateSpeed, 0.15);

    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.MAX_DRIVE_SPEED;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.MAX_DRIVE_SPEED;
    rotateSpeed = rotateLimiter.calculate(rotateSpeed) * DriveConstants.MAX_ROTATE_SPEED;

    if(fieldOriented) {
      if(LimelightHelpers.getTV(LimelightConstants.SHOOTER_SIDE_LIMELIGHT_NAME) || LimelightHelpers.getTV(LimelightConstants.AMP_SIDE_LIMELIGHT_NAME)){
        if (driveWithTargeting){
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotateSpeed, new Rotation2d(swerve.getHeadingWithOffset()));
        }
          } else {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotateSpeed, swerve.getGyro().getRotation2d());
          }
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotateSpeed);
    }

    moduleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    swerve.setModuleStates(moduleStates);

    SmartDashboard.putNumber("Chassis x-speed", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Chassis y-speed", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Chassis rotate-speed", chassisSpeeds.omegaRadiansPerSecond);
    // SmartDashboard.putNumber("Joystick X", joy.getRightX());
    // SmartDashboard.putNumber("Joystick Y", joy.getLeftY());
    // SmartDashboard.putNumber("Joystick Z", joy.getRawAxis(4));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

   // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
