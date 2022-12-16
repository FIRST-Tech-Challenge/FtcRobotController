package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class OwenOpMode extends OpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    DcMotor armMotor;

    Servo rightClaw;
    Servo leftClaw;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        frontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BackLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "BackRight");

        rightClaw = hardwareMap.get(Servo.class, "RightClaw");
        leftClaw =  hardwareMap.get(Servo.class, "LeftClaw");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        armMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        // coneClaw = hardwareMap.get(Servo.class, "Claw");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        int  precision = (gamepad1.right_bumper) ? 3: 1;

        double leftStick = gamepad1.left_stick_y;
        double rightStick = gamepad1.right_stick_y;
        double leftTrigger = gamepad1.right_trigger;
        double rightTrigger = gamepad1.left_trigger;


        double FLeft = 0;
        double FRight = 0;
        double BLeft = 0;
        double BRight = 0;

        if(leftStick != 0 || rightStick != 0) {
            FLeft  = -leftStick;
            FRight = -rightStick;
            BLeft = FLeft;
            BRight = FRight;
        }
        else {
            if (gamepad1.dpad_left) {
                FLeft = -1;
                FRight = 1;
                BRight = -1;
                BLeft = 1;
            }
            else if(gamepad1.dpad_right) {
                FLeft = 1;
                FRight = -1;
                BRight = 1;
                BLeft = -1;
            }
        }
        frontLeftMotor.setPower(FLeft/precision);
        backLeftMotor.setPower(BLeft/precision);
        frontRightMotor.setPower(FRight/precision);
        backRightMotor.setPower(BRight/precision);

        if(gamepad1.b) {
            // rightClaw.setPosition(0.75);
            leftClaw.setPosition(0.8);
        }
        if(gamepad1.x) {
            // rightClaw.setPosition(0.25);
            leftClaw.setPosition(0.3);
        }

        armMotor.setPower((leftTrigger - rightTrigger) / precision);
    }
}

