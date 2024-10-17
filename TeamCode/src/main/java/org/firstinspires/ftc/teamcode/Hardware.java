package org.firstinspires.ftc.teamcode;

import java.util.HashMap;
import java.util.HashSet;

import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.*;
import org.firstinspires.ftc.vision.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

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
    private final MecanumWheels WHEELS;
    private final ExtendableArm ARM;

    private final DigitalChannel COLOR_SWITCH;
    private final DigitalChannel SIDE_SWITCH;

//    private final CRServo INTAKE_SERVO;

    public Hardware(OpMode opMode) {
        this.OP_MODE = opMode;
        autoSleepEnabled = true;

        COLOR_SWITCH = null; //OP_MODE.hardwareMap.get(DigitalChannel.class, "color_switch");
        SIDE_SWITCH = null; //OP_MODE.hardwareMap.get(DigitalChannel.class, "side_switch");

        VISION_PORTAL = null; /*new VisionPortal.Builder()
                        .setCamera(OP_MODE.hardwareMap.get(WebcamName.class, "webcam"))
                        .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                        .build();*/

        WHEELS = initWheels();
        ARM = initArm();
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

        return null;
    }
    
    /**
     * Initiate all hardware needed for the WheelsSystem.
     */
    private ExtendableArm initArm() {
        /*
         * Define arm hardware here.
         * e.g. exampleMotor = OP_MODE.hardwareMap.get(DcMotor.class, "example_motor");
         */
        HashMap<String, DcMotor> motors = new HashMap<>();

        HashMap<String, Servo> servos = new HashMap<>();
        servos.put("clawXServo", OP_MODE.hardwareMap.get(Servo.class, "clawXServo"));
        servos.put("clawZServo", OP_MODE.hardwareMap.get(Servo.class, "clawZServo"));

        CRServo intakeServo = OP_MODE.hardwareMap.get(CRServo.class, "intakeServo");

        return new ExtendableArm(motors, servos, intakeServo);
    }

    public MecanumWheels getWheels() { return WHEELS; }

    public ExtendableArm getArm() {
        return ARM;
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
     * Sleeps the robot any motors are running.
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

        // Does nothing if it isn't a LinearOpMode.
        if (linearOp == null) {
            return;
        }

        // Sleep while any of the motors are still running.
        while (motors.stream().anyMatch(DcMotor::isBusy)) {
            linearOp.sleep(1);
        }
    }
}