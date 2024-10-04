package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.hardware.*;

public class Hardware {
    // The opMode to access the hardware map from.
    private final OpMode OP_MODE;

    // Whether the robot will automatically sleep after each command.
    // Only applicable in LinearOpMode.
    private boolean autoSleepEnabled;

    /* Camera variables */
    private final VisionPortal VISION_PORTAL;
    private static final int RESOLUTION_WIDTH = 100;
    private static final int RESOLUTION_HEIGHT = 100;

    /* Robot systems */
    private final Wheels WHEELS;
    private final Arm ARM;

    // Whether the robot is on red or blue team
    private final DigitalChannel COLOR_SWITCH;
    // Whether the robot is far or near
    private final DigitalChannel SIDE_SWITCH;

    public Hardware(OpMode opMode) {
        this.OP_MODE = opMode;
        autoSleepEnabled = true;

        VISION_PORTAL = new VisionPortal.Builder()
                .setCamera(OP_MODE.hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                .build();

        initWheels();
        initArm();
    }

    /**
     * Initiates all hardware needed for the WheelsSystem.
     */
    private void initWheels() {
        /*
         * Define wheels system hardware here.
         * e.g. exampleMotor = opMode.hardwareMap.get(DcMotor.class, "example_motor");
         */

        WHEELS = new Wheels();
    }

    /**
     * Initiate all hardware needed for the WheelsSystem.
     */
    private void initArm() {
        /*
         * Define arm hardware here.
         * e.g. exampleMotor = opMode.hardwareMap.get(DcMotor.class, "example_motor");
         */

        ARM = new Arm();
    }

    public Wheels getWheels() {
        return WHEELS;
    }

    public Arm getArm() {
        return ARM;
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
            return (LinearOpMode) OP_MODE;

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