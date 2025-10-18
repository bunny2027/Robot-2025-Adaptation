// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.nio.ByteBuffer;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;


/** Add your docs here. */
public class TrackedAprilTag implements Sendable, StructSerializable  {
    private int id;
    private double area;
    private double distance;
    private double ambiguity;
    private double yaw;
    private int cameraId;    



    public TrackedAprilTag(int id, double area, double distance, double ambiguity, double yaw, int cameraId){
        this.id = id;
        this.area = area;
        this.distance = distance;
        this.ambiguity = ambiguity;
        this.yaw = yaw;
        this.cameraId = cameraId;

    
    }
    
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Tracked April Tag");
        builder.addIntegerProperty("id", this::getId, null);
        builder.addDoubleProperty("area", this::getArea, null);
        builder.addDoubleProperty("distance", this::getDistance, null);
        builder.addDoubleProperty("ambiguity", this::getAmbiguity, null);
        builder.addDoubleProperty("yaw", this::getYaw, null);
        builder.addIntegerProperty("cameraId", this::getCameraId, null);
    }


    public int getId(){
        return this.id;
    }

    public double getArea(){
        return this.area;
    }   

    public double getDistance(){
        return this.distance;
    }

    public double getAmbiguity(){
        return this.ambiguity;
    }

    public double getYaw(){
        return this.yaw;
    }

    public int getCameraId(){
        return this.cameraId;
    }

    public static final TrackedAprilTagStruct struct =  new TrackedAprilTagStruct();



}
