package org.firstinspires.ftc.team417;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import java.util.*;

@I2cDeviceType
@DeviceProperties(
        name = "SparkFun OTOS Corrected",
        xmlTag = "SparkFunOTOS2",
        description = "SparkFun Qwiic Optical Tracking Odometry Sensor Corrected"
)
public class SparkFunOTOSCorrected extends SparkFunOTOS {
    public SparkFunOTOSCorrected(I2cDeviceSynch deviceClient) {
        super(deviceClient);
    }

    // Modified version of poseToRegs to fix pose setting issue
    // see https://discord.com/channels/225450307654647808/1246977443030368349/1271702497659977760
    @Override
    protected void poseToRegs(byte[] rawData, Pose2D pose, double xyToRaw, double hToRaw) {
        // Convert pose units to raw data
        short rawX = (short) (_distanceUnit.toMeters(pose.x) * xyToRaw);
        short rawY = (short) (_distanceUnit.toMeters(pose.y) * xyToRaw);
        short rawH = (short) (_angularUnit.toRadians(pose.h) * hToRaw);

        // Store raw data in buffer
        rawData[0] = (byte) (rawX & 0xFF);
        rawData[1] = (byte) ((rawX >> 8) & 0xFF);
        rawData[2] = (byte) (rawY & 0xFF);
        rawData[3] = (byte) ((rawY >> 8) & 0xFF);
        rawData[4] = (byte) (rawH & 0xFF);
        rawData[5] = (byte) ((rawH >> 8) & 0xFF);
    }
}