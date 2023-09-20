package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {
    /* Declare OpMode members. */

    // Define Hardware components Motor, Servo, Sensors objects  (Make them private so they can't be accessed externally)
    // NOTE: Make sure the ID's match the configuration
    // Wheel motors
    private DcMotor frontRightMotor;
    private DcMotor rearRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor rearLeftMotor;
    //Arm

    //Intake

    //Others

    // Define Drive constants.  Make them Public so they CAN be used by the calling OpMode
    // TODO: Add Drive Constants as needed such as motor ticks per rev, Motor power, etc.
    // Example: public static final double Drive_POWER    =  0.7 ;

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init(HardwareMap hardwareMap) {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        frontLeftMotor = hardwareMap.dcMotor.get("front_left_motor");
        frontRightMotor = hardwareMap.dcMotor.get("front_right_motor");
        rearLeftMotor = hardwareMap.dcMotor.get("rear_left_motor");
        rearRightMotor = hardwareMap.dcMotor.get("rear_right_motor");

        // Reverse the right side motors.  This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards, reverse the left side instead.
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void setPowers(double frontLeftPower, double frontRightPower, double rearLeftPower, double rearRightPower) {
        // max is the largest motor power (absolute value) or 1.
        // This ensures all powers maintain the same ratio, if one is outside of the range [-1, 1].
        double max = Math.max(Math.abs(frontLeftPower) + Math.abs(frontRightPower) + Math.abs(rearLeftPower) + Math.abs(rearRightPower), 1);
        double fL_Power = frontLeftPower /max;
        double fR_Power = frontRightPower/max;
        double rL_Power = rearLeftPower  /max;
        double rR_Power = rearRightPower /max;

        frontLeftMotor.setPower(fL_Power);
        frontRightMotor.setPower(fR_Power);
        rearLeftMotor.setPower(rL_Power);
        rearRightMotor.setPower(rR_Power);
    }

    public void drive(double y, double x, double rx) {
        double frontLeftPow  = y + x + rx;
        double frontRightPow = y - x - rx;
        double rearLeftPow   = y - x + rx;
        double rearRightPow  = y + x - rx;

        setPowers(frontLeftPow, frontRightPow, rearLeftPow, rearRightPow);
    }

}