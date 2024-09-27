package org.firstinspires.ftc.teamcode.hardware;

import java.util.HashSet;

import com.qualcomm.robotcore.hardware.*;

public class MecanumSystem extends MotorsSystem {
    /* The DcMotors powering the wheels */
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private double wheelGearRatio = 0.0;

    public MecanumSystem(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
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