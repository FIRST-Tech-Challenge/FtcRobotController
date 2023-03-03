package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Project1Hardware {
    DcMotor frontLeft = null, frontRight = null, backLeft = null, backRight = null;
    DcMotorEx arm = null;
    DcMotor horz = null, vert = null;
    Servo bucket = null, bucketAngle = null;
    Servo claw = null, clawAngle = null;
    NormalizedColorSensor colorSensor = null;
    HardwareMap hwmap = null;
    IMU imu;
    double clawAnglePosition = 1;

    public void init(HardwareMap hardwareMap){
        hwmap = hardwareMap;
        frontLeft = hardwareMap.get(DcMotor.class,"FLeft");
        frontRight = hardwareMap.get(DcMotor.class,"FRight");
        backLeft = hardwareMap.get(DcMotor.class,"BLeft");
        backRight = hardwareMap.get(DcMotor.class,"BRight");
        arm = hardwareMap.get(DcMotorEx.class,"ARM");
        bucket = hardwareMap.get(Servo.class, "BUCKET");
        claw = hardwareMap.get(Servo.class, "CLAW");
        bucketAngle = hardwareMap.get(Servo.class, "BUCKET ANGLE");
        clawAngle = hardwareMap.get(Servo.class, "CLAW ANGLE");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "BUCKET COLOR");
        horz = hardwareMap.get(DcMotor.class, "HORZ SLIDE");
        vert = hardwareMap.get(DcMotor.class, "VERT SLIDE");
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );
    }
}
