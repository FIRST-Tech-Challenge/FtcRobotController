package org.firstinspires.ftc.teamcode.hardwareSystems;

import com.qualcomm.robotcore.hardware.*;

import java.util.Arrays;
import java.util.HashSet;

public class MecanumWheels extends Wheels {
    /**
     * Passed into the `MecanumWheels` constructor.
     * Contains all four motors and the motor type.
     */
    public static class MotorParams {
        public final HashSet<DcMotor> motors;

        /* The DcMotors powering the wheels */
        private final DcMotor FRONT_LEFT_MOTOR;
        private final DcMotor FRONT_RIGHT_MOTOR;
        private final DcMotor BACK_LEFT_MOTOR;
        private final DcMotor BACK_RIGHT_MOTOR;

        private final MotorType MOTOR_TYPE;

        public MotorParams(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
            this(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, MotorType.TETRIX_TORQUENADO);
        }

        public MotorParams(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor, MotorType motorType) {
            this.FRONT_LEFT_MOTOR = frontLeftMotor;
            this.FRONT_RIGHT_MOTOR = frontRightMotor;
            this.BACK_LEFT_MOTOR = backLeftMotor;
            this.BACK_RIGHT_MOTOR = backRightMotor;

            motors = new HashSet<>();
            motors.add(frontLeftMotor);
            motors.add(frontRightMotor);
            motors.add(backLeftMotor);
            motors.add(backRightMotor);

            this.MOTOR_TYPE = motorType;
        }
    }

    /* The DcMotors powering the wheels */
    private final DcMotor FRONT_LEFT_MOTOR;
    private final DcMotor FRONT_RIGHT_MOTOR;
    private final DcMotor BACK_LEFT_MOTOR;
    private final DcMotor BACK_RIGHT_MOTOR;

    public MecanumWheels(MotorParams motorParams, double ticksPerInch) {
        super(motorParams.motors, motorParams.MOTOR_TYPE, ticksPerInch);

        this.FRONT_LEFT_MOTOR = motorParams.FRONT_LEFT_MOTOR;
        this.FRONT_RIGHT_MOTOR = motorParams.FRONT_RIGHT_MOTOR;
        this.BACK_LEFT_MOTOR = motorParams.BACK_LEFT_MOTOR;
        this.BACK_RIGHT_MOTOR = motorParams.BACK_RIGHT_MOTOR;

        /*
         * Set the directions of the motors
         * The right and left motors run in opposite directions of each other
         */
        FRONT_LEFT_MOTOR.setDirection(DcMotorSimple.Direction.REVERSE);
        FRONT_RIGHT_MOTOR.setDirection(DcMotorSimple.Direction.FORWARD);
        BACK_LEFT_MOTOR.setDirection(DcMotorSimple.Direction.REVERSE);
        BACK_RIGHT_MOTOR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public MecanumWheels(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
        super(new HashSet<>(Arrays.asList(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor)));

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
        drive(0, drivePower, turn);
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

    /**
     * {@inheritDoc}
     */
    @Override
    public void turn(double degrees) {

    }
}