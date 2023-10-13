package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware {
    final public ElapsedTime timePassed = new ElapsedTime();

    final private double TICKS_PER_MOTOR_REV = 6000;
    final private double DRIVE_GEAR_REDUCTION = 1.0 ;
    final private double WHEEL_DIAMETER_INCHES = 4.0 ;
    final private double TICKS_PER_INCH = (TICKS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/ (WHEEL_DIAMETER_INCHES * 3.1415);
    final private double DRIVE_SPEED = 0.6;
//    final private double TURN_SPEED = 0.5;
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public void init(HardwareMap hardwareMap) {
        try {
            frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeft.setPower(0);
        } catch (Exception e) {
            telemetry.addData("FrontLeftMotor: ", "Error");
            telemetry.update();
        }
        try {
            frontRight = hardwareMap.dcMotor.get("frontRightMotor");
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setPower(0);
        } catch (Exception e) {
            telemetry.addData("FrontRightMotor: ", "Error");
            telemetry.update();
        }
        try {
            backRight = hardwareMap.dcMotor.get("backRightMotor");
            backRight.setDirection(DcMotor.Direction.FORWARD);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setPower(0);
        } catch (Exception e) {
            telemetry.addData("BackRightMotor: ", "Error");
            telemetry.update();
        }
        try {
            backLeft = hardwareMap.dcMotor.get("backLeftMotor");
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setPower(0);
        } catch (Exception e) {
            telemetry.addData("BackLeftMotor: ", "Error");
            telemetry.update();
        }
    }

    public void drive(double power){
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }
    public void drive(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower){
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }
    // for distance: right is positive, left is negative
    public void strafe(double distance, double power) {
        frontLeft.setTargetPosition((int) (distance * TICKS_PER_INCH));
        frontRight.setTargetPosition((int) (-distance * TICKS_PER_INCH));
        backLeft.setTargetPosition((int) (distance * TICKS_PER_INCH));
        backRight.setTargetPosition((int) (-distance * TICKS_PER_INCH));

        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        while (!(frontLeft.getCurrentPosition() <= frontLeft.getTargetPosition() &&
                frontRight.getCurrentPosition() <= frontRight.getTargetPosition() &&
                backRight.getCurrentPosition() <= backRight.getTargetPosition() &&
                backLeft.getCurrentPosition() <= backLeft.getTargetPosition()));


        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    //positive power -> turn right, negative power -> turn left
    public void turn(double distance, double power){
        if(power > 0){
            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontLeft.setTargetPosition((int) (distance * TICKS_PER_INCH));
            backLeft.setTargetPosition((int) (distance * TICKS_PER_INCH));
        } else {
            frontRight.setPower(power);
            backRight.setPower(power);
            frontRight.setTargetPosition((int) (distance * TICKS_PER_INCH));
            backRight.setTargetPosition((int) (distance * TICKS_PER_INCH));
        }

        while (!(frontLeft.getCurrentPosition() <= frontLeft.getTargetPosition() &&
                backLeft.getCurrentPosition() <= backLeft.getTargetPosition() &&
                frontRight.getCurrentPosition() <= frontRight.getTargetPosition() &&
                backRight.getCurrentPosition() <= backRight.getTargetPosition()));


        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
}