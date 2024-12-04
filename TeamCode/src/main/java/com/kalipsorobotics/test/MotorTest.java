package com.kalipsorobotics.test;

import android.util.Log;

import com.kalipsorobotics.actions.DriveAction;
import com.kalipsorobotics.localization.SparkfunOdometry;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
        IMUModule imuModule = new IMUModule(opModeUtilities);
        SparkfunOdometry sparkfunOdometry = new SparkfunOdometry(driveTrain, opModeUtilities, 0, 0, Math.toRadians(0));
        WheelOdometry wheelOdometry = new WheelOdometry(opModeUtilities, driveTrain, imuModule, 0, 0, Math.toRadians(0));
        DriveAction driveAction = new DriveAction(driveTrain);
        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a) {
                driveTrain.setFLeftPower(0.5);
            } else {
                driveTrain.setFLeftPower(0);
            }
            if (gamepad1.b) {
                driveTrain.setFRightPower(0.5);
            } else {
                driveTrain.setFRightPower(0);
            }
            if (gamepad1.x) {
                driveTrain.setBLeftPower(0.5);
            } else {
                driveTrain.setBLeftPower(0);
            }
            if (gamepad1.y) {
                driveTrain.setBRightPower(0.5);
            } else {
                driveTrain.setBRightPower(0);
            }

        }
    }
}