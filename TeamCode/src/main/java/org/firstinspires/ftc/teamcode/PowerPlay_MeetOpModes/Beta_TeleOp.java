package org.firstinspires.ftc.teamcode.PowerPlay_MeetOpModes;

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
            double horizontal = -gamepad1.left_stick_x * speed; //Normal is negative
            double pivot = -gamepad1.right_stick_x * speed; //Normal has negative

            rightFrontMotor.setPower(pivot - vertical + horizontal);
            leftFrontMotor.setPower(pivot + vertical + horizontal);
            rightRearMotor.setPower(pivot - vertical - horizontal);
            leftRearMotor.setPower(pivot + vertical - horizontal);

//            Arm Extension
            double value = gamepad2.right_stick_y;
            if(armExtension.getCurrentPosition() > 0) {
                if(value >= 0) {
                    armExtension.setPower((value * 0.5) + 0.15);
                } else {
                    armExtension.setPower((value * 0.3) + 0.1);
                }
            } else {
                if(value >= 0) {
                    armExtension.setPower((value * 0.5));
                } else {
                    armExtension.setPower((value * 0.3));
                }
            }


            //Gripper
            if(gamepad2.a) {
                gripServo.setPosition(0.15);
            }
            if(gamepad2.b) {
                gripServo.setPosition(0);
            }

            telemetry.addData("Position: ",armExtension.getCurrentPosition());
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
