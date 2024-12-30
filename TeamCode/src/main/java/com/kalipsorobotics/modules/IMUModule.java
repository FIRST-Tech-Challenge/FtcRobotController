package com.kalipsorobotics.modules;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import com.kalipsorobotics.utilities.OpModeUtilities;

public class IMUModule {
    private static IMUModule single_instance = null;

    private final OpModeUtilities opModeUtilities;
    IMU imu;

    private IMUModule(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        setUpHardware();
    }

    public static synchronized IMUModule getInstance(OpModeUtilities opModeUtilities) {
        if (single_instance == null) {
            single_instance = new IMUModule(opModeUtilities);
        }
        return single_instance;
    }

    public static void setInstanceNull() {
        single_instance = null;
    }

    private void setUpHardware() {
        this.imu = opModeUtilities.getHardwareMap().get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        imu.resetYaw();
    }

    public IMU getIMU() {
        return imu;
    }
}
