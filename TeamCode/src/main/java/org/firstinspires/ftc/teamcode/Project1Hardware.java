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


        //flip1 = hardwareMap.get(DcMotor.class, "FLIP1");
        //flip2 = hardwareMap.get(DcMotor.class, "FLIP2");
        claw1 = hardwareMap.get(Servo.class, "CLAW1");
        claw2 = hardwareMap.get(Servo.class, "CLAW2");
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
                                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
    }

    public void setMotorPositions (int FL, int FR, int BL, int BR){
        frontLeft.setTargetPosition(FL);
        frontRight.setTargetPosition(FR);
        backLeft.setTargetPosition(BL);
        backRight.setTargetPosition(BR);
    }

    public void setMotorPowers (double FL, double FR, double BL, double BR){
        frontLeft.setPower(FL);
        frontRight.setPower(FR);
        backLeft.setPower(BL);
        backRight.setPower(BR);
    }

    public void setMotorPowers (double power){
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }


    public void setMode(String mode){
        if (mode == "reset encoder"){
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else if (mode == "run to pos"){
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void walkForward(int ticks, double power){
        setMode("reset encoder");
        int FLpos = ticks;
        int FRpos = ticks;
        int BLpos = ticks;
        int BRpos = ticks;
        setMotorPowers(power);
        setMotorPositions(FLpos, FRpos, BLpos, BRpos);
    }

    public void rotateRight(int ticks, double power) {
        setMode("reset encoder");
        int FLpos = ticks;
        int FRpos = -ticks;
        int BLpos = -ticks;
        int BRpos = ticks;
        setMotorPowers(power);
        setMotorPositions(FLpos, FRpos, BLpos, BRpos);
    }

    public void rotateLeft(int ticks, double power) {
        setMode("reset encoder");
        int FLpos = -ticks;
        int FRpos = ticks;
        int BLpos = ticks;
        int BRpos = -ticks;
        setMotorPowers(power);
        setMotorPositions(FLpos, FRpos, BLpos, BRpos);
    }

    public void IntakeAndReady() {
        claw1.setPosition(0.8);
        claw2.setPosition(1);
        yaw2.setPosition(0.24);
        vert.setTargetPosition(150);
    }

    public void raiseLift(int pos) {
        vert.setTargetPosition(pos);
    }

    public void release(){
        yaw2.setPosition(-0.1);
        claw1.setPosition(0.95);
        claw2.setPosition(0.75);
        vert.setTargetPosition(150);
    }
}
