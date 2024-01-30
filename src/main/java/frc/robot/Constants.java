// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DriveConstants { // ***UPDATE CONSTANTS***

    public static final double TRACK_WIDTH = Units.inchesToMeters(29   ); // *

    public static final double DRIVE_GEAR_RATIO = 1 / 6.75;
    public static final double ROTATE_GEAR_RATIO = 1 / 12.75;

    public static final double MAX_DRIVE_SPEED = 14.5;
    public static final double MAX_ROTATE_SPEED = 4 * Math.PI;

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI;

    public static final double DRIVE_POSITION_CONVERSION = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER;
    public static final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION / 60;

    public static final double ROTATE_POSITION_CONVERSION = ROTATE_GEAR_RATIO * Math.PI * 2;
    public static final double ROTATE_VELOCITY_CONVERSION = ROTATE_POSITION_CONVERSION / 60;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(TRACK_WIDTH / 2, TRACK_WIDTH / 2),
      new Translation2d(TRACK_WIDTH / 2, -TRACK_WIDTH / 2),
      new Translation2d(-TRACK_WIDTH / 2, TRACK_WIDTH / 2),
      new Translation2d(-TRACK_WIDTH / 2, -TRACK_WIDTH / 2)
    );

    public static final double ROTATE_P_VALUE = 0.5;
    public static final double ROTATE_I_VALUE = 0.0;
    public static final double ROTATE_D_VALUE = 0.0;

    // Drive Motor IDs
    private static final int FRONT_LEFT_DRIVE_ID = 1;
    private static final int FRONT_RIGHT_DRIVE_ID = 2;
    private static final int BACK_LEFT_DRIVE_ID = 3;  
    private static final int BACK_RIGHT_DRIVE_ID = 4;

    // Rotate Motor IDs
    private static final int FRONT_LEFT_ROTATE_ID = 5;
    private static final int FRONT_RIGHT_ROTATE_ID = 6  ;
    private static final int BACK_LEFT_ROTATE_ID = 7;
    private static final int BACK_RIGHT_ROTATE_ID = 8;

    // Swerve Modules
    public static final int[] FRONT_LEFT = {FRONT_LEFT_DRIVE_ID, FRONT_LEFT_ROTATE_ID};
    public static final int[] FRONT_RIGHT = {FRONT_RIGHT_DRIVE_ID, FRONT_RIGHT_ROTATE_ID};
    public static final int[] BACK_LEFT = {BACK_LEFT_DRIVE_ID, BACK_LEFT_ROTATE_ID};
    public static final int[] BACK_RIGHT = {BACK_RIGHT_DRIVE_ID, BACK_RIGHT_ROTATE_ID};

    // Offsets
    public static final double BACK_RIGHT_OFFSET = 1.097 + (Math.PI / 2);

    //Rotate PID Controller Constants
    public static final double ROTATION_P = 8.0;
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 3.0;
    public static final double ROTATION_TOLERANCE = Math.toRadians(Math.toRadians(5));
    
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class LimelightConstants{

    //Amp Side Limelight height constants
    public static final double AMP_SIDE_LIMELIGHT_HEIGHT_TO_SPEAKER = 0;
    public static final double AMP_SIDE_LIMELIGHT_HEIGHT_TO_AMP = 0;
    public static final double AMP_SIDE_LIMELIGHT_HEIGHT_TO_SOURCE = 0;

    //Shooter Side Limelight height constants
    public static final double SHOOTER_SIDE_LIMELIGHT_HEIGHT_TO_SPEAKER = 0;
    public static final double SHOOTER_SIDE_LIMELIGHT_HEIGHT_TO_AMP = 0;
    public static final double SHOOTER_SIDE_LIMELIGHT_HEIGHT_TO_SOURCE = 0;

    //Amp Side Apriltag offset constants
    public static final double AMP_SIDE_APRILTAG_DRIVE_OFFSET = 0;
    public static final double AMP_SIDE_APRILTAG_STRAFE_OFFSET = 0;
    public static final double AMP_SIDE_APRILTAG_ROTATE_OFFSET = 0;

    //Shooter Side Apriltag offset constants
    public static final double SHOOTER_SIDE_APRILTAG_DRIVE_OFFSET = 0;
    public static final double SHOOTER_SIDE_APRILTAG_STRAFE_OFFSET = 0;
    public static final double SHOOTER_SIDE_APRILTAG_ROTATE_OFFSET = 0;

    //Ready-to-Shoot offset constants
    public static final double MINIMUM_DISTANCE_FROM_SPEAKER = 0;
    public static final double MAXIMUM_DISTANCE_FROM_SPEAKER = 0;
    public static final double MINIMUM_ANGLE_OFFSET_FROM_SPEAKER = 0;
    public static final double MAXIMUM_ANGLE_OFFSET_FROM_SPEAKER = 0;
    
    //Limelight name constants
    public static final String AMP_SIDE_LIMELIGHT_NAME = "Amp Side Limelight";
    public static final String SHOOTER_SIDE_LIMELIGHT_NAME = "Shooter Side Limelight";
  }

  public static class AutoAlignConstants{

    //X-movement constants
    public static final double DRIVE_PID_P = 0.08;
    public static final double DRIVE_PID_I = 0;
    public static final double DRIVE_PID_D = 0;

    //Y-movement constants
    public static final double STRAFE_PID_P = 0.04;
    public static final double STRAFE_PID_I = 0;
    public static final double STRAFE_PID_D = 0;

    //Rotate movement constants
    public static final double ROTATE_PID_P = 0.01;
    public static final double ROTATE_PID_I = 0;
    public static final double ROTATE_PID_D = 0;

    //Autoalign deadband
    public static final double AUTO_ALIGN_DEADBAND = 0.05;
  }

  public static class fieldConstants{

    public static final double FIELD_WIDTH_IN_METERS = 16.54175;

    public static final double FIELD_LENGTH_IN_METERS = 8.0137;

    public static final Pose2d FLIPPING_POSE = new Pose2d(
      new Translation2d(FIELD_LENGTH_IN_METERS, FIELD_WIDTH_IN_METERS),
      new Rotation2d(Math.PI));

  }
}
