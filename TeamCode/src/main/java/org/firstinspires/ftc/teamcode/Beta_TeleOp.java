package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class Beta_TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor rightFrontMotor = hardwareMap.get(DcMotor.class,"rightFrontMotor");
        DcMotor leftFrontMotor = hardwareMap.get(DcMotor.class,"leftFrontMotor");
        DcMotor rightRearMotor = hardwareMap.get(DcMotor.class,"rightRearMotor");
        DcMotor leftRearMotor = hardwareMap.get(DcMotor.class,"leftRearMotor");

        DcMotorEx armExtension = hardwareMap.get(DcMotorEx.class, "armExtension");
        Servo gripServo = hardwareMap.get(Servo.class,"gripServo");

        RevBlinkinLedDriver ledLights = hardwareMap.get(RevBlinkinLedDriver.class,"ledLights");
        double speed;
        armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive()) {
            //Speed Control
            if (gamepad1.right_bumper) {
                speed = 1;
            } else if (gamepad1.left_bumper) {
                speed = 0.3;
            } else {
                speed = 0.6;
            }

            //Controller Input - Moves Drive Train
            double vertical = gamepad1.left_stick_y * speed;
            double horizontal = gamepad1.left_stick_x * speed;
            double pivot = -gamepad1.right_stick_x * speed; //normal has negative

            rightFrontMotor.setPower(-pivot + vertical + horizontal);
            leftFrontMotor.setPower(-pivot - vertical + horizontal);
            rightRearMotor.setPower(-pivot + vertical - horizontal);
            leftRearMotor.setPower(-pivot - vertical - horizontal);

//            Arm Extension
            double value = gamepad2.right_stick_y;

            if(armExtension.getCurrentPosition() > 100) {
                if(value >= 0) {
                    armExtension.setPower((value * 0.5) + 0.1);
                } else {
                    armExtension.setPower((value * 0.2) + 0.1);
                }
            } else if(armExtension.getCurrentPosition() > -100){
                if(value >= 0) {
                    armExtension.setPower((value * 0.5));
                } else {
                    armExtension.setPower((value * 0.2));
                }
            } else {
                armExtension.setPower(0.6);
            }

            if(gamepad2.x) {
                gripServo.setPosition(0.15);
            }
            if(gamepad2.b) {
                gripServo.setPosition(0);
            }

            //LED Lights - Utilizes
            if (armExtension.getCurrentPosition() > 150) {
                ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if (armExtension.getCurrentPosition() < 150 && armExtension.getCurrentPosition() > -75) {
                ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else
                ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);

            telemetry.addData("Position: ",armExtension.getCurrentPosition());
            telemetry.addData("Servo Position: ",gripServo.getPosition());
            telemetry.addData("Target: ",armExtension.getTargetPosition());
            telemetry.addData("Power: ",armExtension.getPower());
            telemetry.update();
        }

        rightFrontMotor.setPower(0.0);
        leftFrontMotor.setPower(0.0);
        rightRearMotor.setPower(0.0);
        leftRearMotor.setPower(0.0);
        armExtension.setPower(0.0);

    }


}