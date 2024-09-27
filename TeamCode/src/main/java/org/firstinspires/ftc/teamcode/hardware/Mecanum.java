package org.firstinspires.ftc.teamcode.hardware;

import java.util.HashSet;

import com.qualcomm.robotcore.hardware.*;

public class Mecanum extends Wheels {
    /* The DcMotors powering the wheels */
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private double wheelGearRatio = 0.0;

    public Mecanum(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;

        /*
         * Set the directions of the motors
         * The right and left motors run in opposite directions of each other 
         */
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setMotorPower(double wheelPower) {
        this.wheelPower = wheelPower;
    }

    public double getMotorPower() {
        return wheelPower;
    }

    @Override
    public HashSet<DcMotor> getAllMotors() {
        HashSet<DcMotor> motors = new HashSet<>();
        motors.add(frontLeftMotor);
        motors.add(frontRightMotor);
        motors.add(backLeftMotor);
        motors.add(backRightMotor);
        
        return motors;
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
     * @param x      Sideways movement of the robot
     *               Negative values are leftwards, postive values are rightwards
     * @param y      Forwards/backward movement of the robot
     *               Negative values are backwards, positive values are forwards
     * @param turn   Rotation of the robot
     *               Negative is counterclockwise, positive is clockwise
     */
    @Override
    public void drive(double x, double y, double turn) {
        frontLeftMotor.setPower(y - x - turn);
        frontRightMotor.setPower(y + x + turn);
        backLeftMotor.setPower(y + x - turn);
        backRightMotor.setPower(y - x + turn);
    }
}