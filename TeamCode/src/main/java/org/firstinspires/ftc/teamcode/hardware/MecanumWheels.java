package org.firstinspires.ftc.teamcode.hardware;

import java.util.HashSet;

import com.qualcomm.robotcore.hardware.*;

public class MecanumWheels extends Wheels {
    /* The DcMotors powering the wheels */
    private DcMotor FRONT_LEFT_MOTOR;
    private DcMotor FRONT_RIGHT_MOTOR;
    private DcMotor BACK_LEFT_MOTOR;
    private DcMotor BACK_RIGHT_MOTOR;

    private double wheelGearRatio = -1.0;

    public MecanumWheels(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
        super();
        
        this.FRONT_LEFT_MOTOR = frontLeftMotor;
        this.FRONT_RIGHT_MOTOR = frontRightMotor;
        this.BACK_LEFT_MOTOR = backLeftMotor;
        this.BACK_RIGHT_MOTOR = backRightMotor;

        super.motors.add(frontLeftMotor);
        super.motors.add(frontRightMotor);
        super.motors.add(backLeftMotor);
        super.motors.add(backRightMotor);

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
     *             Negative values are leftwards, postive values are rightwards
     * @param y    Forwards/backward movement of the robot
     *             Negative values are backwards, positive values are forwards
     * @param turn Rotation of the robot
     *             Negative is counterclockwise, positive is clockwise
     */
    @Override
    public void drive(double x, double y, double turn) {
        frontLeftMotor.setPower(y - x - turn);
        frontRightMotor.setPower(y + x + turn);
        backLeftMotor.setPower(y + x - turn);
        backRightMotor.setPower(y - x + turn);
    }

    /**
     * {@inheritDoc}
    */
    public void driveDistance(double distance) {
        driveDistance(distance, 0);
    }

    /**
     * Drive the robot a certain distance in two dimensions.
     * 
     * @param forwardDistance  The distance that the robot travels forward in
     *                         inches.
     *                         Positive is forward and negative is backward.
     * @param sidewaysDistance The distance that the robot travels sidways in inches.
     *                         Negative is leftward and positive is righward.
     */
    @Override
    public void driveDistance(double forwardDistance, double sidewaysDistance) {
    }
}