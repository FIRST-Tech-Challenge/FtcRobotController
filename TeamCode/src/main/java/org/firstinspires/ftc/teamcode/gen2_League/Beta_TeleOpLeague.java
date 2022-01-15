package org.firstinspires.ftc.teamcode.gen2_League;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Beta_TeleOpLeague extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class,"frontRightMotor");
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class,"frontLeftMotor");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class,"backRightMotor");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class,"backLeftMotor");

        CRServo carouselSpinner = hardwareMap.get(CRServo.class, "carouselSpinner");

        DcMotor intakeMotor = hardwareMap.get(DcMotor.class,"intakeMotor");
        DcMotor liftControl = hardwareMap.get(DcMotor.class, "liftControl");

        waitForStart();
        while (opModeIsActive()) {

            //Speed Control
            double speed;
            if (gamepad1.right_bumper) {
                speed = 1.0;
            } else if (gamepad1.left_bumper) {
                speed = 0.2;
            } else {
                speed = 0.5;
            }


            //Controller Input - Moves Drive Train
            double vertical = gamepad1.left_stick_y * speed;
            double horizontal = gamepad1.left_stick_x * speed;
            double pivot = gamepad1.right_stick_x * speed;

            frontRightMotor.setPower(-pivot - vertical - horizontal);
            frontLeftMotor.setPower(pivot - vertical + horizontal);
            backRightMotor.setPower(pivot + vertical - horizontal);
            backLeftMotor.setPower(pivot - vertical - horizontal);

            if (gamepad1.right_trigger == 1) {
                frontRightMotor.setPower(-(-pivot - vertical - horizontal));
                frontLeftMotor.setPower(-(pivot - vertical + horizontal));
                backRightMotor.setPower(-(pivot + vertical - horizontal));
                backLeftMotor.setPower(-(pivot - vertical - horizontal));
            }

            double speedSpinner = 0.5;
            if (gamepad1.x) {
                carouselSpinner.setPower(speedSpinner);
            } else if (gamepad1.b) {
                carouselSpinner.setPower(-speedSpinner);
            } else {
                carouselSpinner.setPower(0.0);
            }

            if (gamepad2.a) {

            }
            if (gamepad2.b) {

            }
            if (gamepad2.y) {

            }
            if (gamepad2.x) {

            }
            if (gamepad2.right_trigger == 1) {
                intakeMotor.setPower(0.2);
            } else {
                intakeMotor.setPower(0);
            }

//            Add Arm Extending Reader

            telemetry.addData("Position: ",liftControl.getCurrentPosition());
        }

        frontRightMotor.setPower(0.0);
        frontLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);

    }


}