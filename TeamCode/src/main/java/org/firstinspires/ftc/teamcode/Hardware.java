package org.firstinspires.ftc.teamcode;

import java.util.HashMap;
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

    private final DigitalChannel COLOR_SWITCH;
    private final DigitalChannel SIDE_SWITCH;

    private final CRServo INTAKE_SERVO;

    public Hardware(OpMode opMode) {
        this.OP_MODE = opMode;
        autoSleepEnabled = true;

        WHEELS = initWheels();
        ARM = null; // initArm();
        WEBCAM = null; // new Webcam(OP_MODE.hardwareMap.get(WebcamName.class, "webcam"));

        COLOR_SWITCH = null; //OP_MODE.hardwareMap.get(DigitalChannel.class, "color_switch");
        SIDE_SWITCH = null; //OP_MODE.hardwareMap.get(DigitalChannel.class, "side_switch");

        INTAKE_SERVO = OP_MODE.hardwareMap.get(CRServo.class, "intakeServo");
    }

    /**
     * Initiates all hardware needed for the WheelsSystem.
     */
    private MecanumWheels initWheels() {
        /*
         * Define wheels system hardware here.
         * e.g. exampleMotor = OP_MODE.hardwareMap.get(DcMotor.class, "example_motor");
         */
        // DcMotor frontLeftMotor = OP_MODE.hardwareMap.get(DcMotor.class, "front_left_wheel");
        // DcMotor frontRightMotor = OP_MODE.hardwareMap.get(DcMotor.class, "front_right_wheel");
        // DcMotor backLeftMotor = OP_MODE.hardwareMap.get(DcMotor.class, "back_left_wheel");
        // DcMotor backRightMotor = OP_MODE.hardwareMap.get(DcMotor.class, "back_right_wheel");

        MecanumWheels.MotorParams motorParams = new MecanumWheels.MotorParams(
                null,
                null,
                null,
                null
        );

        
        return new MecanumWheels(motorParams, 100);
    }

    /**
     * Initiate all hardware needed for the WheelsSystem.
     */
    private ExtendableArm initArm() {
        /*
         * Define arm hardware here.
         * e.g. exampleMotor = OP_MODE.hardwareMap.get(DcMotor.class, "example_motor");
         */
        ExtendableArm.MotorParams motorParams = new ExtendableArm.MotorParams(
                null,
                null
        );

        ExtendableArm.ServoParams servoParams = new ExtendableArm.ServoParams(
                null,
                null,
                null
        );

        CRServo intakeServo = OP_MODE.hardwareMap.get(CRServo.class, "intakeServo");

        double gearRatio = 120.0 / 40.0;
        ExtendableArm.RotationParams rotationParams = new ExtendableArm.RotationParams(
                0,
                1080,
                MotorType.TETRIX_TORQUENADO.getTicksPerRotation() / 360.0
                        * gearRatio
        );

        ExtendableArm.ExtensionParams extensionParams = new ExtendableArm.ExtensionParams(
                0,
                1000
        );

        return new ExtendableArm(motorParams, servoParams, intakeServo, rotationParams, extensionParams);
    }

    public CRServo getIntakeServo() {
        return INTAKE_SERVO;
    }

    public MecanumWheels getWheels() {
        return WHEELS;
    }

    public ExtendableArm getArm() {
        return ARM;
    }

    public Webcam getWebCam() {
        return WEBCAM;
    }

    public DigitalChannel getColorSwitch() {
        return COLOR_SWITCH;
    }

    public DigitalChannel getSideSwitch() {
        return SIDE_SWITCH;
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