package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.hardware;

public class CoyotesRobot {
    private OpMode opMode;

    // Whether the robot will automatically sleep after each command
    // Only applicable in LinearOpMode
    private boolean autoSleepEnabled;

    private MeccanumDrive meccanumDrive;
    private ArmSystem armSystem;

    private DigitalChannel positionSwitch;

    public CoyotesRobot(OpMode opMode) {
        this.opMode = opMode;
        autoSleepEnabled = true;

        initDriveSystem();
        initArmSystem();
    }

    /**
     * Initiates all hardware needed for the DriveSystem.
     */
    private void initDriveSystem() {
        /*
         * Define drive hardware here.
         * e.g. exampleMotor = opMode.hardwareMap.get(DcMotor.class, "example_motor");
         */

        driveSystem = new MeccanumDrive();
    }

    /**
     * Initiate all hardware needed for the ArmSystem.
     */
    private void initArmSystem() {
        /*
         * Define arm hardware here.
         * e.g. exampleMotor = opMode.hardwareMap.get(DcMotor.class, "example_motor");
         */

        armSystem = new ArmSystem();
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

        // Does nothing if it isn't a LinearOpMode
        if (linearOp == null) {
            return;
        }

        // Sleep while any of the motors are still running
        while (Arrays.stream(motors).anyMatch(motor -> motor.isBusy())) {
            linearOp.sleep(1);
        }
    }
}