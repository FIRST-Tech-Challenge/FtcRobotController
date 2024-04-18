package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;

public class MecanumDrive extends WheelSystem {
    /* The DcMotors powering the wheels */
    private DcMotor frontLeftWheel;
    private DcMotor frontRightWheel;
    private DcMotor backLeftWheel;
    private DcMotor backRightWheel;

    // How much power the wheels run with (0.0 - 1.0)
    private double wheelPower = 1.0;

    private double wheelGearRatio = 0.0;

    public MecanumDrive(DcMotor frontLeftWheel, DcMotor frontRightWheel, DcMotor backLeftWheel, DcMotor backRightWheel) {
        this.frontLeftWheel = frontLeftWheel;
        this.frontRightWheel = frontRightWheel;
        this.backLeftWheel = backLeftWheel;
        this.backRightWheel = backRightWheel;

        /*
         * Set the directions of the motors
         * The righ and left motors are reveresed of each other 
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