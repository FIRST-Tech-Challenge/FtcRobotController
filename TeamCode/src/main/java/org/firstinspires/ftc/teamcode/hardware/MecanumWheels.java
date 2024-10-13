package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.*;

public class MecanumWheels extends Wheels {
    /* The DcMotors powering the wheels */
    private final DcMotor FRONT_LEFT_MOTOR;
    private final DcMotor FRONT_RIGHT_MOTOR;
    private final DcMotor BACK_LEFT_MOTOR;
    private final DcMotor BACK_RIGHT_MOTOR;


    private final double WHEEL_GEAR_RATIO = -1.0;

    public MecanumWheels(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
        super();
        
        this.FRONT_LEFT_MOTOR = frontLeftMotor;
        this.FRONT_RIGHT_MOTOR = frontRightMotor;
        this.BACK_LEFT_MOTOR = backLeftMotor;
        this.BACK_RIGHT_MOTOR = backRightMotor;

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
     * {@inheritDoc}
     */
    @Override
    public void drive(double drivePower, double turn) {
        drive(0, drivePower, 0);
    }

    /**
     * {@inheritDoc}
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
    @Override
    public void driveDistance(double distance) {
        driveDistance(distance, 0);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void driveDistance(double forwardDistance, double sidewaysDistance) {
    }
}