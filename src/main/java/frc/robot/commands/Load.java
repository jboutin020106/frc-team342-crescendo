// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.IntakeConstants.DESIRED_SPEED;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.SwerveDrive;

public class Load extends Command {
  /** Creates a new Load. */
  public Intake intake; 
  public Outtake outtake;
  public SwerveDrive swerve;

  public Load(Outtake outtake, Intake intake, SwerveDrive swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.outtake = outtake;
    this.swerve = swerve;

    addRequirements(intake, outtake, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.lockWheels();

    outtake.shootVelocity(DESIRED_SPEED);
     
    if(outtake.isUpToSpeed(DESIRED_SPEED)){
      intake.feedShooter();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    outtake.stop();
    intake.stop();
    swerve.goToZero();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
