package org.firstinspires.ftc.teamcode;

import java.util.HashSet;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.*;
import org.firstinspires.ftc.vision.*;

import org.firstinspires.ftc.teamcode.hardwareSystems.*;

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

    public Hardware(OpMode opMode) {
        this.OP_MODE = opMode;
        autoSleepEnabled = true;

        VISION_PORTAL = new VisionPortal.Builder()
                .setCamera(OP_MODE.hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                .build();

        WHEELS = initWheels();
        ARM = initArm();
    }

    /**
     * Initiates all hardware needed for the WheelsSystem.
     */
    private Wheels initWheels() {
        /*
         * Define wheels system hardware here.
         * e.g. exampleMotor = opMode.hardwareMap.get(DcMotor.class, "example_motor");
         */

        return new Wheels();
    }

    /**
     * Initiate all hardware needed for the WheelsSystem.
     */
    private Arm initArm() {
        /*
         * Define arm hardware here.
         * e.g. exampleMotor = opMode.hardwareMap.get(DcMotor.class, "example_motor");
         */

        return new Arm();
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
     * Sleeps the robot any motors are running
     * @noinspection unused
     */
    public void autoSleep() {
        HashSet<DcMotor> allMotors = new HashSet<>(ARM.getMotors());
        allMotors.addAll(WHEELS.getMotors());

        autoSleep(allMotors);
    }        

    /**
     * Sleeps the robot while the given motors are running
     *
     * @param motors the motors that are running
     */
    public void autoSleep(HashSet<DcMotor> motors) {
        LinearOpMode linearOp = getLinearOpMode();

        // Does nothing if it isn't a LinearOpMode
        if (linearOp == null) {
            return;
        }

        // Sleep while any of the motors are still running
        while (motors.stream().anyMatch(DcMotor::isBusy)) {
            linearOp.sleep(1);
        }
    }
}