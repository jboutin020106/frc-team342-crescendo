// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;


public class Climb extends Command {
  /** Creates a new Climb. */
  private Elevator elevator;
  private XboxController joyStick;
  private final double maxInput = 0.50;

  private double initialPosition;

  public Climb(Elevator elevator, XboxController joyStick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.joyStick = joyStick;
    this.elevator = elevator;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Initial Position",elevator.getElevatorEncoder());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator.getClimbMode()){
      double rightJoy = -joyStick.getRightY();
      double speed = MathUtil.applyDeadband(rightJoy, 0.15);
      elevator.raiseElevatorwithSpeed(speed*maxInput);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.holdPosition();
    SmartDashboard.putNumber("End of Position",elevator.getElevatorEncoder());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}