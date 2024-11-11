package com.kalipsorobotics.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TestHanging extends LinearOpMode {

    public void moveLsUp(DcMotor linearSlideMotorOne, DcMotor linearSlideMotorTwo, double inches){

        linearSlideMotorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double inchesToMM = inches * 25.4;
        double TICKSPERMM = 2 * Math.PI * 18 / 145.1;
        double mmToTicks = inchesToMM * TICKSPERMM;
        double targetTicks = linearSlideMotorOne.getCurrentPosition() + mmToTicks;
        double currentTicks = linearSlideMotorOne.getCurrentPosition() * mmToTicks;
        linearSlideMotorOne.setPower(0.5);
        linearSlideMotorTwo.setPower(0.5);

        while(currentTicks < targetTicks){
            linearSlideMotorOne.setPower(0.5);
            linearSlideMotorTwo.setPower(0.5);
            currentTicks = linearSlideMotorOne.getCurrentPosition();
        }

        linearSlideMotorOne.setPower(0);
        linearSlideMotorTwo.setPower(0);

    }

    public void moveLsDown(DcMotor linearSlideMotorOne, DcMotor linearSlideMotorTwo, double inches){

        linearSlideMotorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double inchesToMM = inches * 25.4;
        double TICKSPERMM = 2 * Math.PI * 18 / 145.1;
        double mmToTicks = inchesToMM * TICKSPERMM;
        double targetTicks = linearSlideMotorOne.getCurrentPosition() - mmToTicks;
        double currentTicks = linearSlideMotorOne.getCurrentPosition() * mmToTicks;
        linearSlideMotorOne.setPower(-0.5);
        linearSlideMotorTwo.setPower(-0.5);

        while(currentTicks > targetTicks){
            linearSlideMotorOne.setPower(-0.5);
            linearSlideMotorTwo.setPower(-0.5);
            currentTicks = linearSlideMotorOne.getCurrentPosition();
        }

        linearSlideMotorOne.setPower(0);
        linearSlideMotorTwo.setPower(0);

    }

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor linearSlideOne = hardwareMap.dcMotor.get("linearSlideOne");
        DcMotor linearSlideTwo = hardwareMap.dcMotor.get("linearSlideTwo");

        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.x){
                moveLsUp(linearSlideOne, linearSlideTwo, 25);
            }

        }
    }
}
