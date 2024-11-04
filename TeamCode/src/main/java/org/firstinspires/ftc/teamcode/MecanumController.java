package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Manages the robot's mecanum drivetrain controls
 */
public class MecanumController {
    private final DcMotor frontLeft, frontRight, backLeft, backRight;

    public MecanumController(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    /**
     * First proposal for mecanum wheel
     *
     * @param angle angle (in degrees) at which the robot should drift to
     * @param magnitude how fast or slow the robot should travel, on a scale from zero to one
     */
    public void moveBot1(float angle, float magnitude) {
        // Represents the top left and bottom right motors
        // These motors have wheels that move to the diagonal right when turned
        double diagonalRight;
        // Sets diagonalRight to sin(angle - π/4)
        diagonalRight = Math.sin(Math.toRadians(angle) - Math.PI / 4);

        // Represents the top right and bottom left motors
        // These motors have wheels that move to the diagonal left when turned
        double diagonalLeft;
        // Sets diagonalLeft to sin(angle - 3π/4)
        diagonalLeft = Math.sin(Math.toRadians(angle) - 3 * Math.PI / 4);

        frontLeft.setPower(diagonalRight);
        frontRight.setPower(diagonalLeft);
        backLeft.setPower(diagonalRight);
        backRight.setPower(diagonalLeft);
    }

    public static void singleSpotDrift(float angle) {

    }
}
