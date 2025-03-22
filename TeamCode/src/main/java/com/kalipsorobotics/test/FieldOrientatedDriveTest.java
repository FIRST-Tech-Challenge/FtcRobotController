package com.kalipsorobotics.test;

import android.util.Log;

import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.actions.drivetrain.FieldOrientedDrive;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class FieldOrientatedDriveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        FieldOrientedDrive fieldOrientedDrive = new FieldOrientedDrive(imuModule.imu, driveTrain, opModeUtilities);
        Log.d("Field Oreintated Drive", "initialized");
        waitForStart();
        while (opModeIsActive()) {
            fieldOrientedDrive.drive(gamepad1);
        }
    }
}
