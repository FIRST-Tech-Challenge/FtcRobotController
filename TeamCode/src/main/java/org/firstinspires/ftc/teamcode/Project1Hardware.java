package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import javax.xml.datatype.DatatypeConfigurationException;

public class Project1Hardware {
    DcMotor frontLeft = null, frontRight = null, backLeft = null, backRight = null;
    DcMotorEx arm = null;
    DcMotor horz = null, vert = null;
    DcMotor flip1, flip2;
    Servo claw1, claw2, yaw1, yaw2;
    Servo bucket = null, bucketAngle = null;
    Servo claw = null, clawAngle = null;
    NormalizedColorSensor colorSensor = null;
    HardwareMap hwmap = null;
    IMU imu;

    public void init(HardwareMap hardwareMap){
        hwmap = hardwareMap;
        frontLeft = hardwareMap.get(DcMotor.class,"FLeft");
        frontRight = hardwareMap.get(DcMotor.class,"FRight");
        backLeft = hardwareMap.get(DcMotor.class,"BLeft");
        backRight = hardwareMap.get(DcMotor.class,"BRight");
        //arm = hardwareMap.get(DcMotorEx.class,"ARM");
        //bucket = hardwareMap.get(Servo.class, "BUCKET");            // bucket open/close
        //bucketAngle = hardwareMap.get(Servo.class, "BUCKET ANGLE"); // whole bucket
        //claw = hardwareMap.get(Servo.class, "CLAW");                // intake open/close
        //clawAngle = hardwareMap.get(Servo.class, "CLAW ANGLE");     // whole intake
        //colorSensor = hardwareMap.get(NormalizedColorSensor.class, "BUCKET COLOR");
        //horz = hardwareMap.get(DcMotor.class, "HORZ SLIDE");
        vert = hardwareMap.get(DcMotor.class, "VERT SLIDE");
        imu = hardwareMap.get(IMU.class, "imu");

        flip1 = hardwareMap.get(DcMotor.class, "FLIP1");
        flip2 = hardwareMap.get(DcMotor.class, "FLIP2");
        claw1 = hardwareMap.get(Servo.class, "CLAW1");
        claw1 = hardwareMap.get(Servo.class, "CLAW1");
        yaw1 = hardwareMap.get(Servo.class, "YAW1");
        yaw2 = hardwareMap.get(Servo.class, "YAW2");


        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vert.setDirection(DcMotor.Direction.REVERSE);


        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
    }
}
