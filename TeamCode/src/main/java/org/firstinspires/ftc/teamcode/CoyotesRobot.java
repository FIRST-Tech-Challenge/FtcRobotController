package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.hardware;

public class CoyotesRobot {
    private OpMode opMode;

    // Whether the robot will automatically sleep after each command
    // Will automatically sleep by default
    private boolean autoSleepEnabled;

    private MeccanumDrive meccanumDrive;
    private ArmSystem armSystem;

    public Hardware(OpMode opMode, DriveSystem driveSystem, ArmSystem armSystem) {
        this.opMode = opMode;

        autoSleepEnabled = true;

        init();

        this.driveSystem = driveSystem;
        this.armSystem = armSystem;
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
     *         Else returns null
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

<<<<<<< HEAD
        // Does nothing if it isn't in LinearOpMode
=======
        // Does nothing if it isn't a LinearOpMode
>>>>>>> a7cf7676ffa13b0e970504180cea917fa21c4687
        if (linearOp == null) {
            return;
        }

        // While any of the motors are still running
        while (Arrays.stream(motors).anyMatch(motor -> motor.isBusy())) {
            linearOp.sleep(1);
        }
    }
}