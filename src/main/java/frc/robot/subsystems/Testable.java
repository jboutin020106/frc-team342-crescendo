// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.function.BooleanSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import frc.robot.LimelightHelpers;


public interface Testable {

    public class Connection {

        private String name; //Name of the device being tested
        private BooleanSupplier connectionCheck;

        /**
         * @param name Device name to be displayed on the dashboard in case of a failure
         * @param connectionCheck The connectionCheck function used to see whether a device is connected
         */
        public Connection(String name, BooleanSupplier connectionCheck){

          this.name = name; //Device name to be displayed on dashboard
          this.connectionCheck = connectionCheck; //The function to check whether the device is connected

        }

        /**
         * @param sparkMax
         * @return The connection status of a sparkMax motor controller
         */
        public static Connection sparkMaxConnectionCheck(CANSparkMax sparkMax){

          int deviceId = sparkMax.getDeviceId(); //CAN id
          BooleanSupplier connectionCheck = () -> !sparkMax.getFirmwareString().equals("v0.0.0");
          return new Connection("SparkMax " + Integer.toString(deviceId), connectionCheck);
          
        }

        /**
         * @param navX
         * @return The state of the navX's connection to the roboRio
         */
        public static Connection navXConnectionCheck(AHRS navX){
            return new Connection("NavX", navX::isConnected);
        }

        public static Connection limelightConnectionCheck(NetworkTable limelightTable, String limelightName){

          BooleanSupplier connectionCheck = () -> {
            long ledMode = limelightTable.getEntry("ledMode").getInteger(4);
            return ledMode != (long) 4;
          };

            return new Connection(limelightName, connectionCheck);
          
        }

        public String getName(){
            return name;
        }

        public boolean connected(){
          return connectionCheck.getAsBoolean();
        }
    }

    default public List<Connection> hardwareConnections(){
          
          return List.of(); //Returns an empty list when not implemented

        }
}
