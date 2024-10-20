package org.firstinspires.ftc.teamcode;

import java.util.HashSet;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardwareSystems.*;

public class Hardware {
    // The opMode to access the hardware map from.
    private final OpMode OP_MODE;

    // Whether the robot will automatically sleep after each command.
    // Only applicable in LinearOpMode.
    private boolean autoSleepEnabled;

    /* Robot systems */
    private final MecanumWheels WHEELS;
    private final ExtendableArm ARM;
    private final Webcam WEBCAM;

    public Hardware(OpMode opMode) {
        this.OP_MODE = opMode;
        autoSleepEnabled = true;

        WHEELS = initWheels();
        ARM = initArm();
        WEBCAM = new Webcam(OP_MODE.hardwareMap.get(WebcamName.class, "webcam"));
    }

    /**
     * Initiates all hardware needed for the WheelsSystem.
     */
    private MecanumWheels initWheels() {
        /*
         * Define wheels system hardware here.
         * e.g. exampleMotor = OP_MODE.hardwareMap.get(DcMotor.class, "example_motor");
         */
        /* PLACEHOLDER */
        return new Wheels();
    }

    /**
     * Initiate all hardware needed for the Wheels.
     */
    private ExtendableArm initArm() {
        /*
         * Define arm hardware here.
         * e.g. exampleMotor = OP_MODE.hardwareMap.get(DcMotor.class, "example_motor");
         */
        /* PLACEHOLDER */
        return new Arm();
    }

    /* Replace `Wheels` with the subclass of `Wheels` used this season. */
    public Wheels getWheels() {
        return WHEELS;
    }

    /* Replace `Arm` with the subclass of `Arm` used this season. */
    public Arm getArm() {
        return ARM;
    }

    public Webcam getWebCam() {
        return WEBCAM;
    }

    /**
     * Attempts to cast the OP_MODE to a LinearOpMode.
     * Returns null if it fails.
     *
     * @return a linearOpMode representation of OP_MODE if possible
     * Else returns null
     */
    public LinearOpMode getLinearOpMode() {
        try {
            return (LinearOpMode) OP_MODE;

        } catch (ClassCastException e) {
            return null;
        }
    }

    public boolean getAutoSleepEnabled() {
        return autoSleepEnabled;
    }

    public void setAutoSleepEnabled(boolean autoSleepEnabled) {
        this.autoSleepEnabled = autoSleepEnabled;
    }

    /**
     * Sleeps the robot while any motors are running.
     */
    public void autoSleep() {
        HashSet<DcMotor> allMotors = new HashSet<>(ARM.getMotors());
        allMotors.addAll(WHEELS.getMotors());

        autoSleep(allMotors);
    }

    /**
     * Sleeps the robot while the given motors are running.
     *
     * @param motors The motors that are running.
     */
    public void autoSleep(HashSet<DcMotor> motors) {
        LinearOpMode linearOp = getLinearOpMode();

        // Does nothing if it isn't a LinearOpMode.
        if (linearOp == null) {
            return;
        }

        // Sleep while any of the motors are still running.
        while (motors.stream().anyMatch(DcMotor::isBusy)) {
            linearOp.idle();
        }
    }
}