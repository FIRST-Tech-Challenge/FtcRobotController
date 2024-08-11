/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
package com.qualcomm.hardware.sparkfun;

import java.nio.ByteBuffer;
import java.util.Arrays;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.wilyworks.simulator.framework.WilySparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@I2cDeviceType
@DeviceProperties(
        name = "SparkFun OTOS",
        xmlTag = "SparkFunOTOS",
        description = "SparkFun Qwiic Optical Tracking Odometry Sensor"
)
public class SparkFunOTOS extends WilySparkFunOTOS {
    public SparkFunOTOS(I2cDeviceSynch deviceClient) {
        super(deviceClient);
    }
}
