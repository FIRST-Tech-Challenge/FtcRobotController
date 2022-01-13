package org.firstinspires.ftc.teamcode.gen1_Qualifying;

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
        DcMotor elementIntake = hardwareMap.get(DcMotor.class, "elementIntake");
        Servo trapdoor = hardwareMap.get(Servo.class,"trapdoor");

        Servo gripServo = hardwareMap.get(Servo.class,"gripServo");
        CRServo capArm = hardwareMap.get(CRServo.class,"capArm");
        CRServo armExtension = hardwareMap.get(CRServo.class, "armExtension");

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
            if (gamepad1.a) {
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
            double pivot = -gamepad1.right_stick_x * speed; //Normal has negative

            frontRightMotor.setPower(-pivot + vertical + horizontal);
            frontLeftMotor.setPower(pivot + vertical - horizontal);
            backRightMotor.setPower(pivot + vertical + horizontal);
            backLeftMotor.setPower(pivot - vertical + horizontal);


            //Arm intake
            if (gamepad2.a){
                armMotor.setPosition(0.92); //straight up
            }
            if (gamepad2.x){
                armMotor.setPosition(0.76); //base level
            }
            if (gamepad2.y){
                armMotor.setPosition(0.82); //second level
            }
            if (gamepad2.b){
                armMotor.setPosition(0.87); //third level
            }
            if (gamepad2.right_trigger > 0) {
                elementIntake.setPower(-1);
            }
            if (gamepad2.right_trigger == 0) {
                elementIntake.setPower(0);
            }
            if (gamepad2.left_trigger > 0) {
                elementIntake.setPower(0.8);
            }
            if (gamepad2.left_trigger == 0) {
                elementIntake.setPower(0);
            }
            if (gamepad2.left_bumper) {
                trapdoor.setPosition(1);
            }
            if (gamepad2.right_bumper) {
                trapdoor.setPosition(0.3);
            }

            //Cap Arm
            if (gamepad2.left_stick_y > 0.0) {
                capArm.setPower(0.5);
            }
            if (gamepad2.left_stick_y < 0.0) {
                capArm.setPower(-0.5);
            }
            if (gamepad2.left_stick_y == 0.0) {
                capArm.setPower(0.0);
            }
            if (gamepad2.dpad_left) {
                gripServo.setPosition(0.5);
            }
            if (gamepad2.dpad_right) {
                gripServo.setPosition(0.0);
            }
            if (gamepad2.right_stick_y > 0.0) {
                armExtension.setPower(0.5);
            }
            if (gamepad2.right_stick_y < 0.0) {
                armExtension.setPower(-0.5);
            }
            if (gamepad2.right_stick_y == 0.0) {
                armExtension.setPower(0.0);
            }


        }

        frontRightMotor.setPower(0.0);
        frontLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);

    }


}
