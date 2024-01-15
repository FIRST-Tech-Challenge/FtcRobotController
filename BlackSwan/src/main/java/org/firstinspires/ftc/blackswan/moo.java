package org.firstinspires.ftc.blackswan;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "moo")
public class moo extends LinearOpMode {

    private PIDController controller;

    public static double p = 0.012, i = 0, d = 0.0001;
    public static double f = 0.1;

    public static int target = 0;

    private final double ticks_in_degree = 1425 / 180.0;

    public static int armQuickPosition = 0;

    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        DcMotor slideLeft = hardwareMap.dcMotor.get("slideLeft");
        DcMotor slideRight = hardwareMap.dcMotor.get("slideRight");
        DcMotor clawUp = hardwareMap.dcMotor.get("clawUp");

        Servo turnClaw = hardwareMap.servo.get("turnClaw");
        Servo closeClaw = hardwareMap.servo.get("closeClaw");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        clawUp.setDirection(DcMotorSimple.Direction.REVERSE);

        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double servoPosition = 0.5;

        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);

        turnClaw.setPosition(0.45);

        waitForStart();

        while (opModeIsActive()) {

            int armEncoder = clawUp.getCurrentPosition();
            double pid = controller.calculate(armEncoder, target);
            double ff = Math.cos(Math.toRadians(target / (1425 / 180.0))) * f;

            double liftPower = pid + ff;

            clawUp.setPower(liftPower);

            if (armQuickPosition == 0) {
                target = 0;
            } else if (armQuickPosition == 1) {
                target = 300;
            }

            if(gamepad2.dpad_up){
                armQuickPosition = 0;
            } else if(gamepad2.dpad_down){
                armQuickPosition = 1;
            }

            telemetry.addData("right stick", gamepad1.left_stick_y);

//            forward -backward

//            if (gamepad1.left_stick_y > .7) {
//                backLeft.setPower(-gamepad1.left_stick_y);
//                backRight.setPower(-gamepad1.left_stick_y);
//                frontLeft.setPower(-gamepad1.left_stick_y);
//                frontRight.setPower(-gamepad1.left_stick_y);
//            } else if (gamepad1.left_stick_y < .7) {
//                backLeft.setPower(-gamepad1.left_stick_y);
//                backRight.setPower(-gamepad1.left_stick_y);
//                frontLeft.setPower(-gamepad1.left_stick_y);
//                frontRight.setPower(-gamepad1.left_stick_y);
//            }else {
//                backLeft.setPower(0);
//                backRight.setPower(0);
//                frontLeft.setPower(0);
//                frontRight.setPower(0);
//            }
//
//            if (gamepad1.right_stick_x > -.7) {
//                backLeft.setPower(gamepad1.right_stick_x);
//                backRight.setPower(-gamepad1.right_stick_x);
//                frontLeft.setPower(gamepad1.right_stick_x);
//                frontRight.setPower(-gamepad1.right_stick_x);
//
//            } else {
//                backLeft.setPower(0);
//                backRight.setPower(0);
//                frontLeft.setPower(0);
//                frontRight.setPower(0);
//            }
//            if (gamepad1.right_stick_x < .7) {
//                backLeft.setPower(gamepad1.right_stick_x);
//                backRight.setPower(-gamepad1.right_stick_x);
//                frontLeft.setPower(gamepad1.right_stick_x);
//                frontRight.setPower(-gamepad1.right_stick_x);
//            } else {
//                backLeft.setPower(0);
//                backRight.setPower(0);
//                frontLeft.setPower(0);
//                frontRight.setPower(0);
//                if (gamepad1.left_stick_x < .7) {
//                    backLeft.setPower(-gamepad1.left_stick_x);
//                    backRight.setPower(gamepad1.left_stick_x);
//                    frontLeft.setPower(gamepad1.left_stick_x);
//                    frontRight.setPower(-gamepad1.left_stick_x);
//                } else {
//                    backLeft.setPower(0);
//                    backRight.setPower(0);
//                    frontLeft.setPower(0);
//                    frontRight.setPower(0);
//                }
//                if (gamepad1.left_stick_x > -.7) {
//                    backLeft.setPower(gamepad1.left_stick_x);
//                    backRight.setPower(-gamepad1.left_stick_x);
//                    frontLeft.setPower(-gamepad1.left_stick_x);
//                    frontRight.setPower(gamepad1.left_stick_x);
//                } else {
//                    backLeft.setPower(0);
//                    backRight.setPower(0);
//                    frontLeft.setPower(0);
//                    frontRight.setPower(0);
//                }
//            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (Math.abs(y) < 0.2) {
                y = 0;
            }
            if (Math.abs(x) < 0.2) {
                x = 0;
            }

            double leftFrontPower = y + x + rx;
            double leftRearPower = y - x + rx;
            double rightFrontPower = y - x - rx;
            double rightRearPower = y + x - rx;

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

            frontLeft.setPower(leftFrontPower);
            backLeft.setPower(leftRearPower);
            frontRight.setPower(rightFrontPower);
            backRight.setPower(rightRearPower);

            telemetry.addData("left slide", slideLeft.getCurrentPosition());
            telemetry.addData("right slide", slideRight.getCurrentPosition());
            if (gamepad2.left_stick_y < -.7 && slideLeft.getCurrentPosition() < 1000 && slideRight.getCurrentPosition() < 1000) {
                slideLeft.setPower(.5);
                slideRight.setPower(.5);
            } else if (gamepad2.left_stick_y > 0.7 && slideLeft.getCurrentPosition() > -5 && slideRight.getCurrentPosition() > -5) {
                slideLeft.setPower(-0.5);
                slideRight.setPower(-0.5);
            } else {
                slideLeft.setPower(0);
                slideRight.setPower(0);
            }


            if (gamepad2.x) {
                turnClaw.setPosition(0.3);
            }
            if (gamepad2.y) {
                turnClaw.setPosition(0.1);
            }
            if (gamepad2.a) {
                closeClaw.setPosition(0.7);
            }
            if (gamepad2.b) {
                closeClaw.setPosition(0.45);
            }
//            if (gamepad2.right_stick_y > 0.7) {
//                clawUp.setPower(0.5);
//            } else {
//                clawUp.setPower(0);
//            }
//            if (gamepad2.right_stick_y < -0.7) {
//                clawUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                clawUp.setPower(-0.5);
//            } else if (gamepad2.right_stick_y > 0.7) {
//                clawUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                clawUp.setPower(0.5);
//            } else {
//                int pos= clawUp.getCurrentPosition();
//                clawUp.setTargetPosition(pos);
//                clawUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                 clawUp.setPower(0.5);
//            }
            telemetry.update();

        }


    }
}