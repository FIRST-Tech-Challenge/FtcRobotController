package org.firstinspires.ftc.teamcode.swift;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class ServoControlOpModeDS extends LinearOpMode{

    private Servo launcherServo = null;

    @Override
    public void runOpMode() {

        waitForStart();

        launcherServo = hardwareMap.get(Servo.class, "launcherServo");

            while (opModeIsActive()) {
                // check to see if we need to move the servo.
                if(gamepad1.left_bumper) {
                    // move to 0 degrees.
                    launcherServo.setPosition(0.5);
                } else if (gamepad1.right_bumper || gamepad1.left_bumper) {
                    // move to 90 degrees.
                    launcherServo.setPosition(1);
                }
                telemetry.addData("Servo Position", launcherServo.getPosition());
                telemetry.addData("Status", "Running");
                telemetry.update();

            }


            telemetry.update();
         }
     }
