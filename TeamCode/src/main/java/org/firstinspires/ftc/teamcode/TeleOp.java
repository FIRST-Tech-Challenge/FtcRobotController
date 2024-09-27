package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp")
public class TeleOp extends OpMode {
    private Hardware hardware;

    // Whether the kill switch has been pressed once within the past 500 ms
    private boolean killSwitchPressedOnce;
    // Times how long it has been since the inital press on the back button
    private ElapsedTime killSwitchTimer = new ElapsedTime();

    @Override
    public void init() {
        hardware = new Hardware(this);

        killSwitchPressedOnce = false;
    }

    @Override
    public void loop() {
        /*
         * Drive robot based on joystick input from gamepad1
         * Right stick moves the robot forwards, backwards and sideways.
         * Left stick rotates it.
         */
        hardware.getMecanumSystem().drive(
                gamepad1.right_stick_x,
                gamepad1.right_stick_y,
                gamepad1.left_stick_x
        );

        // Initial press on kill switch
        if (gamepad1.back || gamepad2.back && !killSwitchPressedOnce) {
            killSwitchPressedOnce = true;
            // Restart the timer
            killSwitchTimer.reset();

        } else if (gamepad1.back || gamepad2.back && !killSwitchPressedOnce) {
            // Second press on kill switch
            killAllMotors();

        } else if (killSwitchTimer.milliseconds() > 500) {
            // If it has been more than 500 since the back button was pressed
            killSwitchPressedOnce = false;
        }
    }

    /**
     * Stop all motors and servos from moving.
     */
    public void killAllMotors() {
        // Text output to log is persistent, unliked telemetry.addData()
        telemetry.log().add("KILL SWITCH HAS BEEN ACTIVATED!");

    }
}