package org.firstinspires.ftc.teamcode.Debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class limit_Slide extends LinearOpMode {

    DcMotor slide, pivot;

    Servo intakeL, wrist;

    int limitSlide, limitPivot;

    private DigitalChannel limitSwitch;

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        initSlide();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad2.left_trigger >= 0.5) {
                intakeL.setPosition(1);
            } else if (gamepad2.right_trigger >= 0.5) {
                intakeL.setPosition(0);
            }

            setSlide(-gamepad2.right_stick_y);
            setPivot(-gamepad2.left_stick_y);
            telemetry.addData("Limit switch: ", limitSwitch.getState());
            telemetry.addData("Encoder count: ", pivot.getCurrentPosition());
            telemetry.update();
        }
    }

    public void initRobot() {
        slide = hardwareMap.get(DcMotor.class, "slide");
        pivot = hardwareMap.get(DcMotor.class, "pivot");

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide.setPower(.00);
        pivot.setPower(.00);

        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        pivot.setDirection(DcMotorSimple.Direction.FORWARD);

        limitSlide = 4750;
        limitPivot = 3200;

        //servos
        intakeL = hardwareMap.get(Servo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");

        intakeL.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.FORWARD);

        wrist.setPosition(0.7);

        //limit switch and brings pivot back

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limit switch");
    }

    public void initSlide() {
        double i = -0.75;
        while (limitSwitch.getState()) {
            pivot.setPower(i);
            telemetry.addData("pivot", pivot.getCurrentPosition());
            telemetry.addLine("initializing slide");
            telemetry.update();
        }
        pivot.setPower(.00);
        sleep(250);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setPower(.00);
    }

    public void setSlide(double x) {
        telemetry.addData("set slide x 1", x);
        if (slide.getCurrentPosition() >= limitSlide && x > 0) {
            x = 0;
        } else if (slide.getCurrentPosition() <= -limitSlide && x < 0) {
            x = 0;
        }

        if (x > 1 && !(slide.getCurrentPosition() > slideLimit()))
            x = .5;
        if (x < -1)
            x = -.5;
        // TODO: add limit for slide based on pivot and have a forced move if necessary
        slide.setPower(x);
        telemetry.addData("set slide x 2", x);
    }

    public double slideLimit() {
        double limit = 0;
        double encoderCountsPerInch = 65; //needs adjusting
        double slideLength = slide.getCurrentPosition() / encoderCountsPerInch;
        limit = Math.toDegrees(Math.acos(46 / slideLength));
        return limit;
    }

    public void setPivot(double x) {
        telemetry.addData("set pivot x 1", x);
        if (pivot.getCurrentPosition() >= limitPivot && x > 0) {
            x = 0;
        } else if (pivot.getCurrentPosition() <= 0 && x < 0) {
            x = 0;
        }

        if (x > 1)
            x = .5;
        if (x < -1)
            x = -.5;

        pivot.setPower(x);
        telemetry.addData("set pivot x 2", x);
    }
}
