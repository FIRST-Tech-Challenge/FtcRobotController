package com.kalipsorobotics.test;

import android.content.Context;
import android.os.Environment;

import com.kalipsorobotics.localization.OdometrySpark;
import com.kalipsorobotics.localization.OdometrySpark;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;

@TeleOp(name = "Fresh OdometryFuse")
public class TestOdometryFuse  extends LinearOpMode {

    SparkFunOTOS myOtos;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor leftBack;

    @Override
    public void runOpMode() throws InterruptedException {
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sprk sensor OTOS");
        rightFront = hardwareMap.get(DcMotor.class, "fRight");
        rightBack = hardwareMap.get(DcMotor.class, "bRight");
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap ,this, telemetry);
        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
        OdometrySpark odometrySpark = new OdometrySpark(myOtos, rightFront, rightBack);
        telemetry.addData("" + odometrySpark.configureOtos(myOtos), "");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            driveTrain.setPower(-gamepad1.right_stick_y);
            telemetry.addData("x: ", odometrySpark.pointCollectData().getX());
            telemetry.addData("y: ", odometrySpark.pointCollectData().getY());
            telemetry.addData("H: ", odometrySpark.headingUpdateData("right", 0, 0));
            telemetry.update();
        }
    }
}