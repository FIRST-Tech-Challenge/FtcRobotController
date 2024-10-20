package org.firstinspires.ftc.teamcode.Debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class clawSlideImport extends LinearOpMode {

    DcMotor slide, pivot;

    int limitSlide, limitPivot;

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        waitForStart();

        while (opModeIsActive()) {
            setSlide(-gamepad2.right_stick_y);
            setPivot(-gamepad2.left_stick_y);
        }
    }

    public void initRobot() {
        slide = hardwareMap.get(DcMotor.class, "slide");
        pivot = hardwareMap.get(DcMotor.class, "pivot");

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide.setPower(.01);
        pivot.setPower(.01);

        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        pivot.setDirection(DcMotorSimple.Direction.FORWARD);

        limitSlide = 4268;
        limitPivot = 2905;
    }

    public void setSlide(double x) {
        if (slide.getCurrentPosition() < limitSlide) {
            if (x < 0)
                x = 0;
        }
        else if (slide.getCurrentPosition() > -limitSlide) {
            if (x < 0)
                x = 0;
        }
        slide.setPower(x);
    }

    public void setPivot(double x) {
        if (pivot.getCurrentPosition() < limitPivot) {
            if (x < 0)
                x = 0;
        }
        else if (pivot.getCurrentPosition() > -limitPivot) {
            if (x > 0)
                x = 0;
        }
        pivot.setPower(x);
    }
}
