package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.*;
import org.firstinspires.ftc.vision.*;

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
    private final Mecanum MECANUM;
    private final Arm ARM;

    // Whether the robot is on red or blue team
    private final DigitalChannel COLOR_SWITCH;
    // Whether the robot is far or near
    private final DigitalChannel SIDE_SWITCH;

    public Hardware(OpMode opMode) {
        this.OP_MODE = opMode;
        autoSleepEnabled = true;

        COLOR_SWITCH = OP_MODE.hardwareMap.get(DigitalChannel.class, "color_switch");
        SIDE_SWITCH = OP_MODE.hardwareMap.get(DigitalChannel.class, "side_switch");

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
         * e.g. exampleMotor = OP_MODE.hardwareMap.get(DcMotor.class, "example_motor");
         */
        DcMotor frontLeftMotor = OP_MODE.hardwareMap.get("front_left_wheel");
        DcMotor frontRightMotor = OP_MODE.hardwareMap.get("front_right_wheel");
        DcMotor backLeftMotor = OP_MODE.hardwareMap.get("back_left_wheel");
        DcMotor backRightMotor = OP_MODE.hardwareMap.get("back_right_wheel");

        MECANUM = new Mecanum(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
    }
    
    /**
     * Initiate all hardware needed for the WheelsSystem.
     */
    private void initArm() {
        /*
         * Define arm hardware here.
         * e.g. exampleMotor = OP_MODE.hardwareMap.get(DcMotor.class, "example_motor");
         */

        ARM = new Arm();
    }

    public Mecanum getWheels() {
        return MECANUM;  
    }

    public Arm getArm() {
        return ARM;
    }

    public DigitalChannel getColorSwitch() {
        return COLOR_SWITCH;
    }

    public DigitalChannel getSideSwitch() {
        return SIDE_SWITCH;
    }

    /**
     * Attempts to cast the OP_MODE to a LinearOpMode
     * Returns null if it fails
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
     * Sleeps the robot any motors are running
     */
    public void autoSleep() {
        autoSleep(super.motors);
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