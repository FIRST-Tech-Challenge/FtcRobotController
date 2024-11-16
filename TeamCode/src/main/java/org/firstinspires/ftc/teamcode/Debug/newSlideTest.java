package org.firstinspires.ftc.teamcode.Debug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp
public class newSlideTest extends LinearOpMode {

    DcMotor slide1, slide2;

    @Override
    public void runOpMode() throws InterruptedException{
        initRobot();
        waitForStart();
        while (opModeIsActive()) {
            slide1.setPower(-gamepad1.right_stick_y);
            slide2.setPower(-gamepad1.left_stick_y);
        }
    }

    public void initRobot() {
        slide1 = hardwareMap.get(DcMotor.class,"slide");
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide1.setDirection(DcMotorSimple.Direction.FORWARD);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide1.setPower(0);

        slide2 = hardwareMap.get(DcMotor.class,"slide 2");
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide2.setDirection(DcMotorSimple.Direction.FORWARD);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setPower(0);
    }
}
