// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.subsystems.Testable.Connection;

/** Add your docs here. */
public class Limelight extends SubsystemBase{

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight table");

    private String limelightName;

    public Limelight(String limelightName){

        this.limelightName = limelightName;

    }

    public double calculateHorizontalDistanceToSpeaker(String limelightName){
        if(LimelightHelpers.getTV(limelightName) == true){
            if(limelightName.equals(LimelightConstants.AMP_SIDE_LIMELIGHT_NAME)){
                double horizontalDistanceToSpeaker = LimelightConstants.AMP_SIDE_LIMELIGHT_HEIGHT_TO_SPEAKER / Math.tan(LimelightHelpers.getTY(limelightName));
                return horizontalDistanceToSpeaker;
             } 

        else if (limelightName.equals(LimelightConstants.SHOOTER_SIDE_LIMELIGHT_NAME)) {
        double horizontalDistanceToSpeaker = LimelightConstants.SHOOTER_SIDE_LIMELIGHT_HEIGHT_TO_SPEAKER / 
        Math.tan(LimelightHelpers.getTY(limelightName));
        return horizontalDistanceToSpeaker;
            } 
        }
            return 0.0;
        }



    public double calculateHorizontalDistanceToAmp(String limelightName){

        if(LimelightHelpers.getTV(limelightName) == true){
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
             }
    return 0.0;
}

    public double calculateHorizontalDistanceToSource(String limelightName){
        if(LimelightHelpers.getTV(limelightName)){
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
      }
        return 0.0;
    }

    public boolean readyToScoreInSpeaker(){
        boolean readyToScoreInSpeaker = false;

        if(LimelightHelpers.getTV(LimelightConstants.SHOOTER_SIDE_LIMELIGHT_NAME)){
            if(LimelightHelpers.getFiducialID(LimelightConstants.SHOOTER_SIDE_LIMELIGHT_NAME) == LimelightConstants.SPEAKER_CENTER_APRILTAG_ID){
                if(LimelightConstants.MINIMUM_DISTANCE_FROM_SPEAKER <= calculateHorizontalDistanceToSpeaker(LimelightConstants.SHOOTER_SIDE_LIMELIGHT_NAME)
                    && LimelightConstants.MAXIMUM_DISTANCE_FROM_SPEAKER >= calculateHorizontalDistanceToSpeaker(LimelightConstants.SHOOTER_SIDE_LIMELIGHT_NAME)){
                         if(LimelightConstants.MINIMUM_ANGLE_OFFSET_FROM_SPEAKER <= LimelightHelpers.getTY(LimelightConstants.SHOOTER_SIDE_LIMELIGHT_NAME) 
                              && LimelightConstants.MAXIMUM_ANGLE_OFFSET_FROM_SPEAKER >= LimelightHelpers.getTY(LimelightConstants.SHOOTER_SIDE_LIMELIGHT_NAME)){
                                    readyToScoreInSpeaker = true;
                   }
                }
            }
        }
            return readyToScoreInSpeaker;
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

    @Override
    public void initSendable(SendableBuilder sendableBuilder){
        sendableBuilder.setSmartDashboardType(limelightName + " Values");
        sendableBuilder.addDoubleProperty(limelightName + " X-Offset", () -> LimelightHelpers.getTX(limelightName), null);
        sendableBuilder.addDoubleProperty(limelightName + " Y-Offset", () -> LimelightHelpers.getTY(limelightName), null);
        sendableBuilder.addDoubleProperty(limelightName + " Horizontal Distance to Speaker", () -> calculateHorizontalDistanceToSpeaker(limelightName), null);
        sendableBuilder.addDoubleProperty(limelightName + " Horizontal Distance to Amp", () -> calculateHorizontalDistanceToAmp(limelightName), null);
        sendableBuilder.addDoubleProperty(limelightName + " Horizontal Distance to Source", () -> calculateHorizontalDistanceToSource(limelightName), null);
        sendableBuilder.addBooleanProperty(limelightName + " Has Target", () -> LimelightHelpers.getTV(limelightName), null);
        sendableBuilder.addBooleanProperty("Can score in speaker?", () -> readyToScoreInSpeaker(), null);
        sendableBuilder.addBooleanProperty("Can score in amp?", () -> canScoreInAmp(), null);
    }
 }



