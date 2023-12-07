package org.firstinspires.ftc.masters.tests;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@TeleOp(name="Demo library", group = "competition")
public class TestTeleOp extends LinearOpMode {

    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftRearMotor = null;
    DcMotor rightRearMotor = null;

    Servo clawServo = null;
    Servo heightControl = null;
    Servo v4b= null;

    Servo droneTipping = null;
    Servo droneLaunch = null;

    double servoPos = .5;
    double v4bPos = .5;

    double droneStartPos = .6;
    double hookStartPos = .45;
    double droneAim = .3;
    double droneRelease = .1;


    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontMotor = hardwareMap.dcMotor.get("frontLeft");
        rightFrontMotor = hardwareMap.dcMotor.get("frontRight");
        leftRearMotor = hardwareMap.dcMotor.get("backLeft");
        rightRearMotor = hardwareMap.dcMotor.get("backRight");

//        droneTipping = hardwareMap.servo.get("droneTip");
//        droneLaunch = hardwareMap.servo.get("droneHook");

//        clawServo = hardwareMap.servo.get("servo");
//        heightControl = hardwareMap.servo.get("intake");
//        v4b = hardwareMap.servo.get("v4b");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double y = 0;
        double x = 0;
        double rx = 0;

//        droneTipping.setPosition(droneStartPos);
//        droneLaunch.setPosition(hookStartPos);

        waitForStart();

        while (opModeIsActive()) {

            y = gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
            //rx = (gamepad1.right_trigger - gamepad1.left_trigger);

            if (Math.abs(y) < 0.2) {
                y = 0;
            }
            if (Math.abs(x) < 0.2) {
                x = 0;
            }

            double leftFrontPower = -y + x + rx;
            double leftRearPower = -y - x + rx;
            double rightFrontPower = -y - x - rx;
            double rightRearPower = -y + x - rx;

            if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 || Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1) {

                double max;
                max = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
                max = Math.max(max, Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(rightRearPower));

                leftFrontPower /= max;
                leftRearPower /= max;
                rightFrontPower /= max;
                rightRearPower /= max;
            }

            leftFrontMotor.setPower(leftFrontPower );
            leftRearMotor.setPower(leftRearPower);
            rightFrontMotor.setPower(rightFrontPower);
            rightRearMotor.setPower(rightRearPower);
//
//            if (gamepad1.a) {
//                droneTipping.setPosition(droneAim);
//            }
//
//            if (gamepad1.b) {
//                droneTipping.setPosition(droneAim);
//                sleep(1000);
//                droneLaunch.setPosition(droneRelease);
//            }
//
//            if (gamepad1.y) {
//                droneTipping.setPosition(droneStartPos);
//                droneLaunch.setPosition(hookStartPos);
//            }

//            if (gamepad1.dpad_up && servoPos < 1) {
//                servoPos+=.005;
//            } else if (gamepad1.dpad_down && servoPos > 0) {
//                servoPos-=.005;
//            }

//            heightControl.setPosition(servoPos);
//
//            if (gamepad1.dpad_left) {
//                intakeFriend.setPower(1);
//            } else if (gamepad1.dpad_right) {
//                intakeFriend.setPower(-1);
//            } else {
//                intakeFriend.setPower(0);
//            }
            //a open claw 0.87
            //b close claw 0.61

//            if (gamepad1.a) {
//                clawServo.setPosition(.87); //open
//            } else if (gamepad1.b) {
//                clawServo.setPosition(.61); //close
//            }

//            if (gamepad1.right_bumper && v4bPos < 1) {
//                v4bPos+=.005;
//            } else if (gamepad1.left_bumper  && v4bPos > 0) {
//                v4bPos-=.005;
//            }
//            v4b.setPosition(v4bPos);
//            telemetry.addData("v4b", v4bPos);
            telemetry.addData("servo pos: ", servoPos);
            telemetry.update();

        }
    }
}
