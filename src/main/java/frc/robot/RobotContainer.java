// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Drive.DriveWithJoystick;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;

import org.opencv.dnn.Net;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private SwerveDrive swerve;

  private Limelight ampSideLimelight;
  private Limelight shooterSideLimelight;

  private XboxController joy;
  private DriveWithJoystick driveWithJoystick;
  private JoystickButton toggleFieldOrientedBtn;
  private JoystickButton toggleDriveWithTargetingBtn;

  private final NetworkTable hardware = NetworkTableInstance.getDefault().getTable("Hardware");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    swerve = new SwerveDrive();
    ampSideLimelight = new Limelight(LimelightConstants.AMP_SIDE_LIMELIGHT_NAME);
    shooterSideLimelight = new Limelight(LimelightConstants.SHOOTER_SIDE_LIMELIGHT_NAME);
    
    joy = new XboxController(0);
    driveWithJoystick = new DriveWithJoystick(swerve, joy, swerve.getFieldOriented(), swerve.getDriveWithTargeting());
    
    swerve.setDefaultCommand(driveWithJoystick);
    toggleFieldOrientedBtn = new JoystickButton(joy, XboxController.Button.kA.value);
    toggleDriveWithTargetingBtn = new JoystickButton(joy, XboxController.Button.kB.value);

    SmartDashboard.putData(swerve);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    toggleFieldOrientedBtn.onTrue(swerve.toggleFieldOriented());
    toggleDriveWithTargetingBtn.onTrue(swerve.toggleDriveWithTargeting());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  private Command checkHardwareCommand(){

    return Commands.sequence(

      new InstantCommand(
          () -> {hardware.getEntry("Drive System").setString(swerve.hardwareConnections()); },
          swerve
      ),

      new InstantCommand(
        () -> {hardware.getEntry("Amp Side Limelight").setString(ampSideLimelight.hardwareConnections()); },
        ampSideLimelight )
      );

      new InstantCommand(
        () -> {hardware.getEntry("Shooter Side Limelight").setString(shooterSideLimelight.hardwareConnections()); }, 
        shooterSideLimelight );
  }
}
