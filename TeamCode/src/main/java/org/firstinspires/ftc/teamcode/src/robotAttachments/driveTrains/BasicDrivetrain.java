package org.firstinspires.ftc.teamcode.src.robotAttachments.driveTrains;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.src.utills.MiscUtils;

/**
 * BasicDrivetrain
 * Implements basic drive functions that can be inherited by other drive systems.
 */
public class BasicDrivetrain {

    /**
     * Front Right Motor Object
     */
    protected DcMotor front_right;

    /**
     * Front Left Motor Object
     */
    protected DcMotor front_left;

    /**
     * Back Right Motor Object
     */
    protected DcMotor back_right;

    /**
     * Back Left Motor Object
     */
    protected DcMotor back_left;


    /**
     * A constructor that takes strings and a hardware map to set up the drive train internally
     *
     * @param hardwareMap The hardware map object from the OpMode class
     * @param frontRight  The name of the front right motor
     * @param frontLeft   The name of the front left motor
     * @param backRight   The name of the back right motor
     * @param backLeft    The name of the back left motor
     */
    public BasicDrivetrain(HardwareMap hardwareMap, String frontRight, String frontLeft, String backRight, String backLeft) {
        front_right = hardwareMap.dcMotor.get(frontRight);
        front_left = hardwareMap.dcMotor.get(frontLeft);
        back_right = hardwareMap.dcMotor.get(backRight);
        back_left = hardwareMap.dcMotor.get(backLeft);

        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * A constructor that takes DcMotor objects. It trusts that the motors are initialized properly
     *
     * @param front_right A DcMotor object tied to the front right motor
     * @param front_left  A DcMotor object tied to the front left motor
     * @param back_right  A DcMotor object tied to the back right motor
     * @param back_left   A DcMotor object tied to the back left motor
     */
    public BasicDrivetrain(DcMotor front_right, DcMotor front_left, DcMotor back_right, DcMotor back_left) {
        this.back_left = back_left;
        this.back_right = back_right;
        this.front_right = front_right;
        this.front_left = front_left;
    }

    /**
     * A empty constructor for subclass initialization
     */
    protected BasicDrivetrain() {
    }


    /**
     * It sets the motors to turn right
     *
     * @param power The power to turn right at
     */
    public void turnRight(double power) {
        front_right.setPower(-power);
        back_right.setPower(-power);
        front_left.setPower(power);
        back_left.setPower(power);
    }

    /**
     * It sets the motors to turn left
     *
     * @param power The power to turn left at
     */
    public void turnLeft(double power) {
        front_right.setPower(power);
        back_right.setPower(power);

        front_left.setPower(-power);
        back_left.setPower(-power);
    }

    /**
     * This resets the motors to their proper configuration
     */
    public void reinitializeMotors() {
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Stops all the motors
     */
    public void stopAll() {
        back_right.setPower(0);
        back_left.setPower(0);
        front_right.setPower(0);
        front_left.setPower(0);
    }

    /**
     * It strafes at the given angle with the provided power
     *
     * @param angle The angle to strafe at
     * @param power The power to strafe at
     */
    public void strafeAtAngle(double angle, double power) {
        power = MiscUtils.boundNumber(power);
        double power1;
        double power2;

        angle = angle % 360;

        power1 = -Math.cos(Math.toRadians(angle + 45.0));
        power2 = -Math.cos(Math.toRadians(angle - 45));

        power1 = power * power1;
        power2 = power * power2;

        front_right.setPower(power1);
        back_left.setPower(power1);

        front_left.setPower(power2);
        back_right.setPower(power2);

    }

}
