package com.kalipsorobotics.test.outtake;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TestLinearSlide2 extends LinearOpMode {
    DcMotor linearSlideMotor1;
    DcMotor linearSlideMotor2;
    OpModeUtilities opModeUtilities;
    @Override
    public void runOpMode() throws InterruptedException{
        linearSlideMotor1 = hardwareMap.dcMotor.get("linearSlide1");
        linearSlideMotor2 = hardwareMap.dcMotor.get("linearSlide2");
        linearSlideMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlideMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            linearSlideMotor1.setPower(gamepad2.right_stick_y);
            linearSlideMotor2.setPower(gamepad2.right_stick_y);
            //Both motors are reversed
        }
    }
}
