package com.kalipsorobotics.fresh.test;

import com.kalipsorobotics.fresh.OpModeUtilities;
import com.kalipsorobotics.fresh.mechanism.OuttakeSlide;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class TestLinearSlide extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        //OuttakeSlide linearSlide = new OuttakeSlide(opModeUtilities);

        DcMotor linearSlide = hardwareMap.get(DcMotor.class,"linearSlide");

        waitForStart();
        while (opModeIsActive()) {
//            if (gamepad1.a) {
//                linearSlide.setPower(1);
//            } else if (gamepad1.b) {
//                linearSlide.setPower(-1);
//            } else {
//                linearSlide.setPower(0);
//            }
            linearSlide.setPower(gamepad1.left_stick_y);
        }
    }
}
