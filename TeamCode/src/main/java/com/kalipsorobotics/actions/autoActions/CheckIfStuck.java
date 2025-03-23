package com.kalipsorobotics.actions.autoActions;

import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

public class CheckIfStuck {
    IMU imu;
    DriveTrain driveTrain;
    OpModeUtilities opModeUtilities;
    public CheckIfStuck(IMU imu1, DriveTrain driveTrain1, OpModeUtilities opModeUtilities1) {
        imu = imu1;
        driveTrain = driveTrain1;
        opModeUtilities = opModeUtilities1;
    }

    public boolean checkStuck() {
        DcMotor motor = driveTrain.getfRight();
        boolean isStuck = false;
        return isStuck;
    }
}
