package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.hardware;

public class Hardware {
    private OpMode opMode;

    // Whether the robot will automatically sleep after each command
    // Only applicable in LinearOpMode
    private boolean autoSleepEnabled;

    private Mecanum mecanum;
    private Arm arm;

    // Whether the robot is on red or blue team
    private DigitalChannel colorSwitch;
    // Whether the robot is far or near
    private DigitalChannel sideSwitch;

    public Hardware(OpMode opMode) {
        this.opMode = opMode;
        autoSleepEnabled = true;

        colorSwitch = opMode.hardwareMap.get(DigitalChannel.class, "color_switch");
        sideSwitch = opMode.hardwareMap.get(DigitalChannel.class, "side_switch");

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
        DcMotor frontLeftMotor = opMode.hardwareMap.get("front_left_wheel");
        DcMotor frontRightMotor = opMode.hardwareMap.get("front_right_wheel");
        DcMotor backLeftMotor = opMode.hardwareMap.get("back_left_wheel");
        DcMotor backRightMotor = opMode.hardwareMap.get("back_right_wheel");

        mecanum = new Mecanum(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
    }
    
    /**
     * Initiate all hardware needed for the WheelsSystem.
     */
    private void initArm() {
        /*
         * Define arm hardware here.
         * e.g. exampleMotor = opMode.hardwareMap.get(DcMotor.class, "example_motor");
         */

        arm = new Arm();
    }

    public Wheels getWheels() {
        return wheels;
    }

    public Arm getArm() {
        return arm;
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