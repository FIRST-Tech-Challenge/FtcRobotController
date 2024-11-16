package com.kalipsorobotics.test;

import com.kalipsorobotics.localization.OdometryFuse;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
        OdometryFuse odometryFuse = new OdometryFuse(myOtos, rightFront, rightBack);
        telemetry.addData("" + odometryFuse.configureOtos(myOtos), "");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            driveTrain.setPower(-gamepad1.right_stick_y);
            telemetry.addData("x: ", odometryFuse.pointCollectData().getX());
            telemetry.addData("y: ", odometryFuse.pointCollectData().getY());
            telemetry.addData("H: ", odometryFuse.headingUpdateData("right"));
            telemetry.update();
        }
    }
}