package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Beta_TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class,"frontRightMotor");
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class,"frontLeftMotor");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class,"backRightMotor");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class,"backLeftMotor");

        CRServo rightSpinner = hardwareMap.get(CRServo.class,"rightSpinner");
        CRServo leftSpinner = hardwareMap.get(CRServo.class,"leftSpinner");

        Servo armMotor = hardwareMap.get(Servo.class,"armMotor");

        waitForStart();
        while (opModeIsActive()) {

            //Speed Control
            double speed;
            if (gamepad1.right_bumper) {
                speed = 1;
            } else if (gamepad1.left_bumper) {
                speed = 0.3;
            } else {
                speed = 0.5;
            }

            //Carousel Spinner
            if (gamepad2.a) {
                double speedSpinner = 0.5;
                rightSpinner.setPower(speedSpinner);
                leftSpinner.setPower(-speedSpinner);
            } else {
                rightSpinner.setPower(0.0);
                leftSpinner.setPower(0.0);
            }

            //Controller Input - Moves Drive Train
            double vertical = gamepad1.left_stick_y * speed;
            double horizontal = gamepad1.left_stick_x * speed; //Normal is positive
            double pivot = -gamepad1.right_stick_x * speed; //Normalhas negative

            frontRightMotor.setPower(pivot + vertical + horizontal);
            frontLeftMotor.setPower(pivot + vertical - horizontal);
            backRightMotor.setPower(pivot - vertical + horizontal);
            backLeftMotor.setPower(pivot + vertical + horizontal);

        }

        frontRightMotor.setPower(0.0);
        frontLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);

    }


}
