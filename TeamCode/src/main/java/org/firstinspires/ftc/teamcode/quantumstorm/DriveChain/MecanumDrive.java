package org.firstinspires.ftc.teamcode.quantumstorm.DriveChain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/**
 * Four-motor mecanum drive chain.
 * Usage: create in init, then call drive(y, x, rx) every loop.
 */
public class MecanumDrive {

    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    // Last powers sent to the motors, for telemetry
    private double frontLeftPower = 0;
    private double frontRightPower = 0;
    private double backLeftPower = 0;
    private double backRightPower = 0;

    public MecanumDrive(HardwareMap hardwareMap) {

        // =========================================================
        // HARDWARE MAP
        // =========================================================

        frontLeft = hardwareMap.get(DcMotor.class, Constants.FRONT_LEFT_DRIVE);
        frontRight = hardwareMap.get(DcMotor.class, Constants.FRONT_RIGHT_DRIVE);
        backLeft = hardwareMap.get(DcMotor.class, Constants.BACK_LEFT_DRIVE);
        backRight = hardwareMap.get(DcMotor.class, Constants.BACK_RIGHT_DRIVE);

        // =========================================================
        // MOTOR DIRECTIONS
        // =========================================================

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // =========================================================
        // ZERO POWER BEHAVIOR
        // =========================================================

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stop();
    }

    /**
     * Robot-centric mecanum drive.
     *
     * @param y  forward (+) / backward (-)
     * @param x  strafe right (+) / left (-)
     * @param rx turn clockwise (+) / counter-clockwise (-)
     */
    public void drive(double y, double x, double rx) {

        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;

        // Normalize powers so no wheel exceeds 1.0
        double maxPower = Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));

        if (maxPower > 1.0) {
            fl /= maxPower;
            fr /= maxPower;
            bl /= maxPower;
            br /= maxPower;
        }

        setPowers(fl, fr, bl, br);
    }

    /**
     * Field-centric mecanum drive: the stick moves the robot relative to the
     * driver, whichever way the robot is facing.
     *
     * @param y          away from the driver (+) / toward the driver (-)
     * @param x          driver's right (+) / left (-)
     * @param rx         turn clockwise (+) / counter-clockwise (-)
     * @param headingRad robot heading from the IMU, counter-clockwise positive,
     *                   0 = facing away from the driver
     */
    public void driveFieldCentric(double y, double x, double rx, double headingRad) {

        // Rotate the stick vector by -heading to turn it into robot directions
        double cos = Math.cos(-headingRad);
        double sin = Math.sin(-headingRad);

        double robotX = x * cos - y * sin;
        double robotY = x * sin + y * cos;

        // Correct the chassis' forward direction in robot directions, after rotating
        drive(robotY * Constants.FIELD_CENTRIC_FORWARD_SIGN, robotX, rx);
    }

    /** Set each wheel directly. Used by drive() and by MecanumDriveTest. */
    public void setPowers(double fl, double fr, double bl, double br) {

        frontLeftPower = fl;
        frontRightPower = fr;
        backLeftPower = bl;
        backRightPower = br;

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    public void stop() {
        setPowers(0, 0, 0, 0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Front Left", "%.2f", frontLeftPower);
        telemetry.addData("Front Right", "%.2f", frontRightPower);
        telemetry.addData("Back Left", "%.2f", backLeftPower);
        telemetry.addData("Back Right", "%.2f", backRightPower);
    }
}
