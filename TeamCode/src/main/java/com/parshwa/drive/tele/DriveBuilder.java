package com.parshwa.drive.tele;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

public class DriveBuilder {
    private DcMotorSimple.Direction MotorOrientationRightFront = DcMotor.Direction.FORWARD;
    private DcMotorSimple.Direction MotorOrientationRightBack  = DcMotor.Direction.FORWARD;
    private DcMotorSimple.Direction MotorOrientationLeftFront  = DcMotor.Direction.REVERSE;
    private DcMotorSimple.Direction MotorOrientationLeftBack   = DcMotor.Direction.REVERSE;
    private double speed                                       = 0.5;
    private DriveModes driverMode                              = DriveModes.MecanumRobotOriented;
    private String imuName                                         = "imu";
    private String rightFront                                  = "RightFrontMotor";
    private String rightBack                                   = "RightBackMotor";
    private String leftFront                                   = "LeftFrontMotor";
    private String leftBack                                    = "LeftBackMotor";
    private IMU imu;


    public DriveBuilder speed(double speed){
        this.speed = speed;
        return this;
    }
    public DriveBuilder imuName(String imu){
        this.imuName = imu;
        return this;
    }
    public DriveBuilder imu(IMU imu){
        this.imu = imu;
        return this;
    }
    public DriveBuilder motors(String rightFront, String rightBack, String leftFront, String leftBack){
        this.rightFront = rightFront;
        this.rightBack = rightBack;
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        return this;
    }
    public DriveBuilder mode(DriveModes mode){
        this.driverMode = mode;
        return this;
    }
    public DriveBuilder oreintaion(DcMotorSimple.Direction MotorOrientationRightFront, DcMotorSimple.Direction MotorOrientationRightBack, DcMotorSimple.Direction MotorOrientationLeftFront, DcMotorSimple.Direction MotorOrientationLeftBack){
        this.MotorOrientationRightFront = MotorOrientationRightFront;
        this.MotorOrientationRightBack = MotorOrientationRightBack;
        this.MotorOrientationLeftFront = MotorOrientationLeftFront;
        this.MotorOrientationLeftBack = MotorOrientationLeftBack;
        return this;
    }

    public DcMotorSimple.Direction getMotorOrientationRightFront() {
        return MotorOrientationRightFront;
    }

    public DcMotorSimple.Direction getMotorOrientationRightBack() {
        return MotorOrientationRightBack;
    }

    public DcMotorSimple.Direction getMotorOrientationLeftFront() {
        return MotorOrientationLeftFront;
    }

    public DcMotorSimple.Direction getMotorOrientationLeftBack() {
        return MotorOrientationLeftBack;
    }

    public double getSpeed() {
        return speed;
    }

    public DriveModes getDriverMode() {
        return driverMode;
    }

    public String getImuName() {
        return imuName;
    }
    public IMU getImu(){
        return imu;
    }
    public String getRightFront() {
        return rightFront;
    }

    public String getRightBack() {
        return rightBack;
    }

    public String getLeftFront() {
        return leftFront;
    }

    public String getLeftBack() {
        return leftBack;
    }
}
