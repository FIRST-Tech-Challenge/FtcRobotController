package com.kalipsorobotics.modules;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import com.kalipsorobotics.utilities.OpModeUtilities;

public class IMUModule {
    private static IMUModule single_instance = null;

    private OpModeUtilities opModeUtilities;
    IMU imu;

    private IMUModule(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        setUpHardware();

        imu.resetYaw();
    }

    public static synchronized IMUModule getInstance(OpModeUtilities opModeUtilities) {
        if (single_instance == null) {
            single_instance = new IMUModule(opModeUtilities);
        } else {
            single_instance.opModeUtilities = opModeUtilities;
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

    }

    public IMU getIMU() {
        return imu;
    }
}
