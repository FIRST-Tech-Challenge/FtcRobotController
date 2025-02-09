package com.kalipsorobotics.modules;

import android.provider.ContactsContract;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import com.kalipsorobotics.utilities.OpModeUtilities;

public class IMUModule {
    private static IMUModule single_instance = null;

    private OpModeUtilities opModeUtilities;
    public IMU imu;

    private IMUModule(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;

        resetHardwareMap(opModeUtilities.getHardwareMap(), this);
        int i = 0;
        while (i < 3) {
            i++;
            boolean isImuIntitalized = imu.initialize(new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
            ));
            if (isImuIntitalized) {
                break;
            } else {
                opModeUtilities.getOpMode().sleep(500);
            }
        }

        imu.resetYaw();
    }

    public static synchronized IMUModule getInstance(OpModeUtilities opModeUtilities) {
        if (single_instance == null) {
            single_instance = new IMUModule(opModeUtilities);
        } else {
            resetHardwareMap(opModeUtilities.getHardwareMap(), single_instance);
        }
        return single_instance;
    }

    public static void setInstanceNull() {
        single_instance = null;
    }

    private static void resetHardwareMap(HardwareMap hardwareMap, IMUModule imuModule) {
        imuModule.imu = hardwareMap.get(IMU.class, "imu");
    }

    public IMU getIMU() {
        return imu;
    }
}
