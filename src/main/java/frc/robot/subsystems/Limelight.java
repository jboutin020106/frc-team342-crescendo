// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

/** Add your docs here. */
public class Limelight extends SubsystemBase {

    private String limelightName;

    public Limelight(String limelightName){

        this.limelightName = limelightName;

    }

    public double calculateHorizontalDistanceToSpeaker(){

        double horizontalDistanceToSpeaker = LimelightConstants.LIMELIGHT_HEIGHT_TO_SPEAKER / Math.tan(LimelightHelpers.getTY(limelightName));
        return horizontalDistanceToSpeaker;

    }

    public double calculateHorizontalDistanceToAmp(){

        double horizontalDistanceToAmp = LimelightConstants.LIMELIGHT_HEIGHT_TO_AMP / Math.tan(LimelightHelpers.getTY(limelightName));
        return horizontalDistanceToAmp;

    }

    public double calculateHorizontalDistanceToSource(){

        double horizontalDistanceToSource = LimelightConstants.LIMELIGHT_HEIGHT_TO_SOURCE / Math.tan(LimelightHelpers.getTY(limelightName));
        return horizontalDistanceToSource;

    }
}


