package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class CORobotAutoCodeLM1_V1 extends LinearOpMode {
    @Override
public void runOpMode() throws InterruptedException {
            // Declare our motors
            // Make sure your ID's match your configuration
            DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
            DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
            DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
            DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
            Servo specsServo = hardwareMap.get(Servo.class, "specsServo");
            DcMotor rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");

            specsServo.setPosition(1);
            sleep(1000);
            frontLeftMotor.setPower(-0.2);
            frontRightMotor.setPower(-0.2);
            backRightMotor.setPower(-0.2);
            backLeftMotor.setPower(-0.2);
            sleep(1000);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            sleep(1000);
            rightSlideMotor.setPower(0.5);
            sleep(1000);
            rightSlideMotor.setPower(-0.5);
            specsServo.setPosition(0);
            sleep(1000);
            rightSlideMotor.setPower(0);
            sleep(1000);
            frontLeftMotor.setPower(-0.5);
            frontRightMotor.setPower(0.5);
            backLeftMotor.setPower(0.5);
            backRightMotor.setPower(-0.5);
            sleep(1000);
            frontLeftMotor.setPower(0.5);
            frontRightMotor.setPower(0.5);
            backRightMotor.setPower(0.5);
            backLeftMotor.setPower(0.5);
            sleep(1000);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            backLeftMotor.setPower(0);
        }
    }