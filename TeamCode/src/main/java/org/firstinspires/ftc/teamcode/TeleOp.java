package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp")
public class TeleOp extends OpMode {
    private Hardware hardware;

    // The current state of the killSwitch
    private killSwitchState killSwitchState;
    // Times how long it has been since the inital press on the back button
    private ElapsedTime killSwitchTimer = new ElapsedTime();

    @Override
    public void init() {
        hardware = new Hardware(this);

        killSwitchState = KillSwitchState.OFF;
    }

    @Override
    public void loop() {
        checkKillSwitch();
        
        // Prevent any actions if the kill switch is on
        if (killSwitchState == KillSwitchState.ON) {
            return;
        }

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

    /**
     * Handle the kill switch.
     * If back is pressed once, the kill switch is `PRIMED`
     * If it is hit twice, the kill switch is `ON`
     * If is `PRIMED` but has not been hit again for more than 500 ms,
     * set the kill switch to `OFF` again.
     */
    public void checkKillSwitch() {
        // Initial press on kill switch
        if (gamepad1.back || gamepad2.back && killSwitchState == KillSwitchState.OFF) {
            killSwitchState = KillSwitchState.PRIMED;
            // Restart the timer
            killSwitchTimer.reset();

        } else if (gamepad1.back || gamepad2.back && killSwitchState == KillSwitchState.PRIMED) {
            // Second press on kill switch
            killSwitchState = KillSwitchState.ON;
            killAllMotors();

        } else if (killSwitchTimer.milliseconds() > 500 && killSwitchState == KillSwitchState.PRIMED) {
            // If it has been more than 500 since the back button was pressed
            killSwitchState = KillSwitchState.OFF;
        }
    }

    /**
     * Stop all motors and servos from moving.
     */
    public void killAllMotors() {
        // Text output to log is persistent, unliked telemetry.addData()
        telemetry.log().add("KILL SWITCH HAS BEEN ACTIVATED!");

        wheels.getAllMotors().forEach(motor -> motor.setPower(0));
        arm.getAllMotors().forEach(motor -> motor.setPower(0));
    }
}