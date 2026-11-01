package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp
public class TwoControllerMecanumTeleOp extends LinearOpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor outtake;
    DcMotor intake;
    DcMotor transfer;

    CRServo left;
    CRServo right;

    VoltageSensor voltage;
    Servo gate;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        outtake = hardwareMap.get(DcMotor.class, "Outtake");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        transfer = hardwareMap.get(DcMotor.class, "Transfer");
        left = hardwareMap.get(CRServo.class, "left");
        right = hardwareMap.get(CRServo.class, "right");
        voltage = hardwareMap.get(VoltageSensor.class, "Control Hub");
        gate = hardwareMap.get(Servo.class, "gate");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double power;
        double mPower;
        double volts = voltage.getVoltage();
        double vChange = 1;



        mPower = 1;

        power = 0.75;


        boolean leftBumper = false;
        boolean rightBumper = false;
        boolean isBelowVoltage = false;


        waitForStart();

        if(isStopRequested()) return;


        while(opModeIsActive()) {


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator * mPower;
            double backLeftPower = (y - x + rx) / denominator * mPower;
            double frontRightPower = (y - x - rx) / denominator * mPower;
            double backRightPower = (y + x - rx) / denominator * mPower;

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            if (gamepad2.left_bumper && !leftBumper) {
                power += 0.05;

            } else if (gamepad2.right_bumper && !rightBumper) {
                power -= 0.05;
            }
            if (volts < 12) {
                isBelowVoltage = true;
                vChange *= Math.pow(13/volts, 2);
                frontLeftPower *= volts/13;
                backLeftPower *= volts/13;
                frontRightPower *= volts/13;
                backRightPower *= volts/13;

            } else {
                vChange = 1;
            }

            leftBumper = gamepad2.left_bumper;
            rightBumper = gamepad2.right_bumper;

            if (gamepad2.left_trigger_pressed) {
                outtake.setPower(power);
                transfer.setPower(1);
            } else {
                outtake.setPower(0);
                transfer.setPower(0);
            }
            if (gamepad1.right_trigger_pressed) {
                mPower = 0.4;
            } else {
                mPower = 1;
            }


            /*
            controls:
            A: Intake transfer max speed
            B: Transfer slow
            X: Reverse
            Left Trigger: Outtake only
            Right Trigger: slow transfer
             */

            if (gamepad2.a) {
                intake.setPower(1);
                left.setPower(1);
                right.setPower(-1);
                transfer.setPower(0.8);
            } else {
                intake.setPower(0);
                transfer.setPower(0);
                left.setPower(0);
                right.setPower(0);

            }
            if (gamepad2.b) {
                gate.setPosition(150);
                transfer.setPower(0.8);
            } else {
                gate.setPosition(30);
                transfer.setPower(0);
            }
            if (gamepad2.x) {
                transfer.setPower(-1);
                intake.setPower(-1);
                left.setPower(-1);
                right.setPower(1);
            }



            telemetry.addLine("its running");
            telemetry.addData("power", power);
            telemetry.addData("isBelowVoltage", isBelowVoltage);
            telemetry.addData("volts", volts);
            telemetry.update();

            telemetry.update();


        }

    }
}
