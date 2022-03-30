package org.firstinspires.ftc.teamcode.src.robotAttachments.driveTrains;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Adds extra functionality to BasicDrivetrain to be used in Teleop
 */
public class TeleopDriveTrain extends BasicDrivetrain {
    /**
     * A internal variable to scale the input sensitivity
     */
    private double DrivePowerMultiplier;


    /**
     * Constructs drive train from hardware map and motor names
     *
     * @param hardwareMap The hardware map object from the OpMode class
     * @param frontRight  The name of the front right motor
     * @param frontLeft   The name of the front left motor
     * @param backRight   The name of the back right motor
     * @param backLeft    The name of the back left motor
     */
    public TeleopDriveTrain(HardwareMap hardwareMap, String frontRight, String frontLeft, String backRight, String backLeft) {
        super(hardwareMap, frontRight, frontLeft, backRight, backLeft);
        this.DrivePowerMultiplier = 1;
    }


    /**
     * Constructs drive train from hardware map and motor names
     *
     * @param hardwareMap The hardware map object from the OpMode class
     * @param frontRight  The name of the front right motor
     * @param frontLeft   The name of the front left motor
     * @param backRight   The name of the back right motor
     * @param backLeft    The name of the back left motor
     */
    public TeleopDriveTrain(HardwareMap hardwareMap, String frontRight, String frontLeft, String backRight, String backLeft, boolean driveEncoders) {
        front_right = hardwareMap.dcMotor.get(frontRight);
        front_left = hardwareMap.dcMotor.get(frontLeft);
        back_right = hardwareMap.dcMotor.get(backRight);
        back_left = hardwareMap.dcMotor.get(backLeft);

        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        if (driveEncoders) {
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else {
            front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        this.DrivePowerMultiplier = 1;
    }


    /**
     * Allows control of the teleop drivetrain
     *
     * @param gamepad1 The first gamepad
     * @param gamepad2 The second gamepad
     * @return Always returns null
     */
    public Object gamepadControl(Gamepad gamepad1, Gamepad gamepad2) {

        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottom most position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        this.back_left.setPower(-DrivePowerMultiplier * ((gamepad1.left_stick_y + gamepad1.left_stick_x) + gamepad1.right_stick_x));
        this.front_left.setPower(-DrivePowerMultiplier * ((gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x));
        this.back_right.setPower(-DrivePowerMultiplier * ((gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x));
        this.front_right.setPower(-DrivePowerMultiplier * ((gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x));


        //Speed Modifiers
        if (gamepad1.b) {
            this.setDrivePowerMultiplier(0.6);
        }
        if (gamepad1.y) {
            this.setDrivePowerMultiplier(1);

        }
        if (gamepad1.a) {
            this.setDrivePowerMultiplier(0.3);
        }
        return null;
    }

    /**
     * Allows caller to scale the sensitivity of the gamepad
     *
     * @param drivePowerMultiplier The new power scale factor
     */
    public void setDrivePowerMultiplier(double drivePowerMultiplier) {
        this.DrivePowerMultiplier = drivePowerMultiplier;
    }

}
