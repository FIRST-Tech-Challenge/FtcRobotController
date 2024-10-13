package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.*;

public class Mecanum extends Wheels {
    /* The DcMotors powering the wheels */
    private final DcMotor FRONT_LEFT_MOTOR;
    private final DcMotor FRONT_RIGHT_MOTOR;
    private final DcMotor BACK_LEFT_MOTOR;
    private final DcMotor BACK_RIGHT_MOTOR;

    private double WHEEL_GEAR_RATIO = -1.0;

    public Mecanum(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
        FRONT_LEFT_MOTOR = frontLeftMotor;
        FRONT_RIGHT_MOTOR = frontRightMotor;
        BACK_LEFT_MOTOR = backLeftMotor;
        BACK_RIGHT_MOTOR = backRightMotor;

        super.MOTORS.add(FRONT_LEFT_MOTOR);
        super.MOTORS.add(FRONT_RIGHT_MOTOR);
        super.MOTORS.add(BACK_LEFT_MOTOR);
        super.MOTORS.add(BACK_RIGHT_MOTOR);

        /*
         * Set the directions of the motors
         * The right and left motors run in opposite directions of each other
         */
        FRONT_LEFT_MOTOR.setDirection(DcMotorSimple.Direction.REVERSE);
        FRONT_RIGHT_MOTOR.setDirection(DcMotorSimple.Direction.FORWARD);
        BACK_LEFT_MOTOR.setDirection(DcMotorSimple.Direction.REVERSE);
        BACK_RIGHT_MOTOR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Drive the mecanum wheels.
     * 
     * @param drivePower Forwards/backward movement of the robot
     *                   Negative values are backwards, positive values are forwards
     * @param turn       Rotation of the robot
     *                   Negative is counterclockwise, positive is clockwise
     */
    @Override
    public void drive(double drivePower, double turn) {
        drive(0, drivePower, 0);
    }

    /**
     * Drive the mecanum wheels.
     * 
     * @param x    Sideways movement of the robot
     *             Negative values are leftwards, positive values are rightwards
     * @param y    Forwards/backward movement of the robot
     *             Negative values are backwards, positive values are forwards
     * @param turn Rotation of the robot
     *             Negative is counterclockwise, positive is clockwise
     */
    @Override
    public void drive(double x, double y, double turn) {
        FRONT_LEFT_MOTOR.setPower(y - x - turn);
        FRONT_RIGHT_MOTOR.setPower(y + x + turn);
        BACK_LEFT_MOTOR.setPower(y + x - turn);
        BACK_RIGHT_MOTOR.setPower(y - x + turn);
    }

    /**
     * {@inheritDoc}
    */
    public void driveDistance(double distance) {
        drive(distance, 0);
    }

    /**
     * Drive the robot a certain distance in two dimensions.
     * 
     * @param forwardDistance  The distance that the robot travels forward in
     *                         inches.
     *                         Positive is forward and negative is backward.
     * @param sidewaysDistance The distance that the robot travels sideways in inches.
     *                         Negative is leftward and positive is rightward.
     */
    @Override
    public void driveDistance(double forwardDistance, double sidewaysDistance) {
    }
}