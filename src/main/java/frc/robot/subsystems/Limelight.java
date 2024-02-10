// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Testable.Connection;

/** Add your docs here. */
public class Limelight extends SubsystemBase{

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight table");

    private String limelightName;

    public Limelight(String limelightName){

        this.limelightName = limelightName;

    }

    public double calculateHorizontalDistanceToSpeaker(String limelightName){

        if(limelightName.equals(LimelightConstants.AMP_SIDE_LIMELIGHT_NAME)){
        double horizontalDistanceToSpeaker = LimelightConstants.AMP_SIDE_LIMELIGHT_HEIGHT_TO_SPEAKER / Math.tan(LimelightHelpers.getTY(limelightName));
        return horizontalDistanceToSpeaker;
        } 

        else if (limelightName.equals(LimelightConstants.SHOOTER_SIDE_LIMELIGHT_NAME)) {
        double horizontalDistanceToSpeaker = LimelightConstants.SHOOTER_SIDE_LIMELIGHT_HEIGHT_TO_SPEAKER / 
        Math.tan(LimelightHelpers.getTY(limelightName));
        return horizontalDistanceToSpeaker;
        } 
        
        else {
            double horizontalDistanceToSpeaker = 0;
            return horizontalDistanceToSpeaker;
        }


    }

    public double calculateHorizontalDistanceToAmp(String limelightName){

        if(limelightName.equals(LimelightConstants.AMP_SIDE_LIMELIGHT_NAME)){
            double horizontalDistanceToAmp = LimelightConstants.AMP_SIDE_LIMELIGHT_HEIGHT_TO_AMP 
            / Math.tan(LimelightHelpers.getTY(limelightName));
            return horizontalDistanceToAmp;
        }

        else if(limelightName.equals(LimelightConstants.SHOOTER_SIDE_LIMELIGHT_NAME)){
            double horizontalDistanceToAmp = LimelightConstants.SHOOTER_SIDE_LIMELIGHT_HEIGHT_TO_AMP
            / Math.tan(LimelightHelpers.getTY(limelightName));
            return horizontalDistanceToAmp;
        }

        else{
            double horizontalDistanceToAmp = 0;
            return horizontalDistanceToAmp;
        }



    }

    public double calculateHorizontalDistanceToSource(String limelightName){

         if(limelightName.equals(LimelightConstants.AMP_SIDE_LIMELIGHT_NAME)){
            double horizontalDistanceToSource = LimelightConstants.AMP_SIDE_LIMELIGHT_HEIGHT_TO_SOURCE
            / Math.tan(LimelightHelpers.getTY(limelightName));
            return horizontalDistanceToSource;
        }

        else if(limelightName.equals(LimelightConstants.SHOOTER_SIDE_LIMELIGHT_NAME)){
            double horizontalDistanceToSource = LimelightConstants.SHOOTER_SIDE_LIMELIGHT_HEIGHT_TO_AMP
            / Math.tan(LimelightHelpers.getTY(limelightName));
            return horizontalDistanceToSource;
        }

        else{
            double horizontalDistanceToSource = 0;
            return horizontalDistanceToSource;
        }
    }

    public boolean readyToShoot(){
        boolean readyToShoot = false;

        if(LimelightHelpers.getTV(LimelightConstants.SHOOTER_SIDE_LIMELIGHT_NAME)){
            if(LimelightHelpers.getFiducialID(LimelightConstants.SHOOTER_SIDE_LIMELIGHT_NAME) == LimelightConstants.SPEAKER_CENTER_APRILTAG_ID){
                if(LimelightConstants.MINIMUM_DISTANCE_FROM_SPEAKER <= calculateHorizontalDistanceToSpeaker(LimelightConstants.SHOOTER_SIDE_LIMELIGHT_NAME)
                    && LimelightConstants.MAXIMUM_DISTANCE_FROM_SPEAKER >= calculateHorizontalDistanceToSpeaker(LimelightConstants.SHOOTER_SIDE_LIMELIGHT_NAME)){
                         if(LimelightConstants.MINIMUM_ANGLE_OFFSET_FROM_SPEAKER <= LimelightHelpers.getTY(LimelightConstants.SHOOTER_SIDE_LIMELIGHT_NAME) 
                              && LimelightConstants.MAXIMUM_ANGLE_OFFSET_FROM_SPEAKER >= LimelightHelpers.getTY(LimelightConstants.SHOOTER_SIDE_LIMELIGHT_NAME)){
                                    readyToShoot = true;
                   }
                }
            }
        }
        return readyToShoot;
    }

    public boolean canScoreInAmp(){

        boolean canScoreInAmp = false;

        if(LimelightHelpers.getTV(LimelightConstants.AMP_SIDE_LIMELIGHT_NAME)){
            if(LimelightHelpers.getFiducialID(LimelightConstants.AMP_SIDE_LIMELIGHT_NAME) == LimelightConstants.AMP_APRILTAG_ID){
                if(LimelightConstants.MINIMUM_DISTANCE_FROM_AMP <= calculateHorizontalDistanceToSpeaker(LimelightConstants.AMP_SIDE_LIMELIGHT_NAME)
                    && LimelightConstants.MAXIMUM_DISTANCE_FROM_AMP >= calculateHorizontalDistanceToSpeaker(LimelightConstants.AMP_SIDE_LIMELIGHT_NAME)){
                if(LimelightConstants.MINIMUM_ANGLE_OFFSET_FROM_AMP <= LimelightHelpers.getTY(LimelightConstants.AMP_SIDE_LIMELIGHT_NAME) 
                    && LimelightConstants.MAXIMUM_ANGLE_OFFSET_FROM_AMP >= LimelightHelpers.getTY(LimelightConstants.AMP_SIDE_LIMELIGHT_NAME)){
                        canScoreInAmp = true;
                   }
                }
            }
        }
        return canScoreInAmp;
    }

    public List<Connection> hardwareConnections() {
        return List.of(
            Connection.limelightConnectionCheck(table, limelightName)
        );
    }
 }



