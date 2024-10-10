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
        hardware.getWheels().drive(
                gamepad1.right_stick_x,
                gamepad1.right_stick_y,
                gamepad1.left_stick_x);
    }
}