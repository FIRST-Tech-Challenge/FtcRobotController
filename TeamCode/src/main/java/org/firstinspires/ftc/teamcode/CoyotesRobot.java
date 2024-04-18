package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

public class CoyotesRobot {
    private OpMode opMode;

    // Whether the robot will automatically sleep after each command
    // Will automatically sleep by default
    private boolean autoSleepEnabled;

    private DriveSystem driveSystem;

    public Hardware(OpMode opMode) {
        this.opMode = opMode;

        autoSleepEnabled = true;

        init();

        driveSystem = new DriveSystem();
    }

    /**
     * Initializes all the robot's hardware(motors, servos, sensors, etc.)
     */
    public void init() {

    }

    /**
     * Attempts to cast the opMode to a LinearOpMode
     * Returns null if it fails
     *
     * @return a linearOpMode representation of opMode if possible
     * Else returns null
     */
    public LinearOpMode getLinearOpMode() {
        try {
            return (LinearOpMode) opMode;

        } catch (ClassCastException e) {
            return null;
        }
    }

    /**
     * Sleeps the robot while the given motors are running
     *
     * @param motors the motors that are running
     */
    public void autoSleep(DcMotor... motors) {
        LinearOpMode linearOp = getLinearOpMode();

        // does nothing if it isn't a LinearOpMode
        if (linearOp == null) {
            return;
        }

        // while any of the motors are still running
        while (Arrays.stream(motors).anyMatch(motor -> motor.isBusy())) {
            linearOp.sleep(1);
        }
    }
}