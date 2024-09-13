package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.*;

public class MecanumDrive extends DriveSystem {
    /* The DcMotors powering the wheels */
    private DcMotor frontLeftWheel;
    private DcMotor frontRightWheel;
    private DcMotor backLeftWheel;
    private DcMotor backRightWheel;

    private double wheelGearRatio = 0.0;

    public MeccanumDrive(DcMotor frontLeftWheel, DcMotor frontRightWheel, DcMotor backLeftWheel, DcMotor backRightWheel) {
        this.frontLeftWheel = frontLeftWheel;
        this.frontRightWheel = frontRightWheel;
        this.backLeftWheel = backLeftWheel;
        this.backRightWheel = backRightWheel;

        /*
         * Set the directions of the motors
         * The right and left motors are reveresed of each other 
         */
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setWheelPower(double wheelPower) {
        this.wheelPower = wheelPower;
    }

    public double getWheelPower() {
        return wheelPower;
    }

    /**
     * Controls the robot's driving
     * 
     * @param x      Sideways movement of the robot
     *               Negative values are leftwards, postive values are rightwards
     * @param y      Forwards/backward movement of the robot
     *               Negative values are backwards, positive values are forwards
     * @param rotate Rotation of the robot
     *               Negative is counterclockwise, positive is clockwise
     */
    public void drive(double x, double y, double rotate) {
        frontLeftMotor.setPower(y - x - rotate);
        frontRightMotor.setPower(y + x + rotate);
        backLeftMotor.setPower(y + x - rotate);
        backRightMotor.setPower(y - x + rotate);
    }
}