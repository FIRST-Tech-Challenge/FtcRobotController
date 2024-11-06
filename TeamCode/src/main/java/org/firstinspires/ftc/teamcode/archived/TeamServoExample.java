package org.firstinspires.ftc.teamcode.archived;

import android.icu.text.Transliterator;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
//CRServo
// @TeleOp
@Disabled
public class TeamServoExample extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo intakeServo = hardwareMap.get(Servo.class, "cameraServo");
        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        telemetry.addData("Hello",", Team RoboActive");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double armMotorSpeed = 0;
        int armPosition = 0;
        double position = 0.0;

        armMotor.setVelocity(250);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(100);
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        // run until the end of the match (driver presses STOP)

        armMotor.setVelocity(armMotorSpeed);
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("Status", "Game Started...");
            telemetry.update();

            // check to see if we need to move the servo.
            if(gamepad1.y) {
            intakeServo.setPosition(0.1);
            } if (gamepad1.b) {
                intakeServo.setPosition(0.0);
            } if (gamepad1.left_trigger == 1) {
                intakeServo.setPosition(0.29);
            } if (gamepad1.right_trigger == 1) {
                intakeServo.setPosition(0.29);
            }if (gamepad1.left_bumper) {
                sleep(5);
                armPosition = armPosition - 3;
                telemetry.addData("armposition",armPosition);
                telemetry.update();
            } if (gamepad1.right_bumper) {
                sleep(5);
                armPosition = armPosition + 3;
                telemetry.addData("armposition",armPosition);
                telemetry.update();
            }
            armMotor.setTargetPosition(armPosition);
            armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);



            waitForStart();
            telemetry.addData("Servo Position", intakeServo.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}