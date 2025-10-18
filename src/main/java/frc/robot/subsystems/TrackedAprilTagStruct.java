// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

/** Add your docs here. */
public class TrackedAprilTagStruct implements Struct<TrackedAprilTag>{
    @Override
    public Class<TrackedAprilTag> getTypeClass() {
        return TrackedAprilTag.class;
    }

    @Override
    public String getTypeName() {
        return "TrackedAprilTag";
    }

    @Override
    public int getSize() {
        return (kSizeDouble * 4) + (kSizeInt32 * 2);
    }

    @Override
    public String getSchema() {
        return "int32 id;double area;double distance;double ambiguity;double yaw;int32 cameraId;";
    }

    @Override
    public TrackedAprilTag unpack(ByteBuffer bb) {
        return new TrackedAprilTag(bb.getInt(), bb.getDouble(), bb.getDouble(), bb.getDouble(), bb.getDouble(), bb.getInt());
    }

    @Override
    public void pack(ByteBuffer bb, TrackedAprilTag value) {
        bb.putInt(value.getId());
        bb.putDouble(value.getArea());
        bb.putDouble(value.getDistance());
        bb.putDouble(value.getAmbiguity());
        bb.putDouble(value.getYaw());
        bb.putInt(value.getCameraId());
    }
}
