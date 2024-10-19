package org.firstinspires.ftc.teamcode.Debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class clawSlideImport extends LinearOpMode {

    DcMotor slide, pivot;

    int limitSlide, limitPivot;

    @Override
    public void runOpMode() throws InterruptedException {
        try {

            while (opModeIsActive()) {
                slide(-gamepad2.right_stick_y);
                pivot(-gamepad2.left_stick_y);

            }
        } catch (Exception e) {
            telemetry.addData("Error: ", e);
        }
    }

    public void initRobot() {
        DcMotor slide = hardwareMap.get(DcMotor.class, "slide");
        DcMotor pivot = hardwareMap.get(DcMotor.class, "pivot");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide.setPower(0);
        pivot.setPower(0);

        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        pivot.setDirection(DcMotorSimple.Direction.FORWARD);

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limitSlide = 9999;
        limitPivot = 9999;
    }

    public void slide(double x) {

        if (slide.getCurrentPosition() < limitSlide) {
            slide.setPower(0);
        }
        else if (slide.getCurrentPosition() > -limitSlide) {
            slide.setPower(0);
        }

        slide.setPower(x);
    }

    public void pivot(double x) {
        if (pivot.getCurrentPosition() < limitPivot) {
            pivot.setPower(0);
        }
        else if (pivot.getCurrentPosition() > -limitPivot) {
            pivot.setPower(0);
        }

        pivot.setPower(x);

    }
}
