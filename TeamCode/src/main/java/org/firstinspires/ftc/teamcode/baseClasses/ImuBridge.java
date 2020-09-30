package org.firstinspires.ftc.teamcode.baseClasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.jetbrains.annotations.NotNull;

public class ImuBridge {
    static BNO055IMU get(@NotNull HardwareMap hardwareMap) {
        return get(hardwareMap, "imu");
    }

    static BNO055IMU get(@NotNull HardwareMap hardwareMap, String name) {
        return hardwareMap.get(BNO055IMU.class, name);
    }
}
