package com.kalipsorobotics.test;

import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.FieldOrientedDrive;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class FieldOrientatedDriveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        FieldOrientedDrive fieldOrientedDrive = new FieldOrientedDrive(imuModule.imu, driveTrain, opModeUtilities);
        waitForStart();
        while (opModeIsActive()) {
            fieldOrientedDrive.drive(gamepad1);
        }
    }
}
