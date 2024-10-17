package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

@TeleOp(name="TeleOp")
public class DriverMode extends OpMode {
    private Hardware hardware;

    @Override
    public void init() {
        hardware = new Hardware(this);
    }

    @Override
    public void loop() {
        /*
         * Drive robot based on joystick input from gamepad1
         * Right stick moves the robot forwards, backwards and sideways.
         * Left stick rotates it.
         */
//        hardware.getWheels().drive(
//                gamepad1.right_stick_x,
//                gamepad1.right_stick_y,
//                gamepad1.left_stick_x
//        );
//
//        hardware.getArm().rotateArm(gamepad2.right_stick_y);
//        hardware.getArm().extendArm(gamepad2.right_stick_x);
//
//        hardware.getArm().rotateClawXServo(gamepad2.left_stick_x);
//        hardware.getArm().rotateClawZServo(gamepad2.left_stick_y);
//
//        if (gamepad2.dpad_down) {
//            hardware.getArm().startIntake();
//        } else if (gamepad2.dpad_up) {
//            hardware.getArm().stopIntake();
//        }
//
//        // Update the information from the robot
//        telemetry.update();
    }
}