package com.kalipsorobotics.fresh.test;

import com.kalipsorobotics.fresh.OpModeUtilities;
import com.kalipsorobotics.fresh.localization.Odometry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Alan.OdometryFuse;

public class TestOdometryFuse  extends LinearOpMode {
    SparkFunOTOS myOtos;
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap ,this, telemetry);
        OdometryFuse odometryFuse = new OdometryFuse(myOtos, leftFront, leftBack, rightFront, rightBack);
    }
}
