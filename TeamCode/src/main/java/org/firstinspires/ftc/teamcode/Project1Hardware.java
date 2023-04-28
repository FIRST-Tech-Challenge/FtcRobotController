package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    IMU imu1;

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
        imu1 = hardwareMap.get(IMU.class, "imu");


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

        imu1.initialize(
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

    public void setMode(int mode){
        if (mode == 1){
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else if (mode == 2){
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }else if (mode == 3){
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void walkForward(int ticks, double power){
        //setMode(1);
        int FLpos = frontLeft.getCurrentPosition()+ticks;
        int FRpos = frontRight.getCurrentPosition()+ticks;
        int BLpos = backLeft.getCurrentPosition()+ticks;
        int BRpos = backRight.getCurrentPosition()+ticks;
        setMotorPowers(power);
        setMotorPositions(FLpos, FRpos, BLpos, BRpos);
        setMode(2);
    }

    public void walkBackward(int ticks, double power){
        //setMode(1);
        int FLpos = frontLeft.getCurrentPosition()-ticks;
        int FRpos = frontRight.getCurrentPosition()-ticks;
        int BLpos = backLeft.getCurrentPosition()-ticks;
        int BRpos = backRight.getCurrentPosition()-ticks;
        setMotorPowers(power);
        setMotorPositions(FLpos, FRpos, BLpos, BRpos);
        setMode(2);
    }

    public void rotateRight(int ticks, double power) {
        //setMode(1);
        int FLpos = frontLeft.getCurrentPosition()+ticks;
        int FRpos = frontRight.getCurrentPosition()-ticks;
        int BLpos = backLeft.getCurrentPosition()+ticks;
        int BRpos = backRight.getCurrentPosition()-ticks;
        setMotorPowers(power);
        setMotorPositions(FLpos, FRpos, BLpos, BRpos);
        setMode(2);
    }

    public void rotateLeft(int ticks, double power) {
        //setMode(1);
        int FLpos = frontLeft.getCurrentPosition()+ticks;
        int FRpos = frontRight.getCurrentPosition()-ticks;
        int BLpos = backLeft.getCurrentPosition()+ticks;
        int BRpos = backRight.getCurrentPosition()-ticks;
        setMotorPowers(power);
        setMotorPositions(FLpos, FRpos, BLpos, BRpos);
        setMode(2);
    }

    public void moveLeft(int ticks, double power1, double power2, double power3, double power4) {
        //setMode(1);
        int FLpos = frontLeft.getCurrentPosition()-ticks;
        int FRpos = frontRight.getCurrentPosition()+ticks;
        int BLpos = backLeft.getCurrentPosition()+ticks;
        int BRpos = backRight.getCurrentPosition()-ticks;
        setMotorPowers(power1, power2, power3, power4);
        setMotorPositions(FLpos, FRpos, BLpos, BRpos);
        setMode(2);
    }

    public void moveRight(int ticks, double power1, double power2, double power3, double power4) {
        //setMode(1);
        int FLpos = frontLeft.getCurrentPosition()+ticks;
        int FRpos = frontRight.getCurrentPosition()-ticks;
        int BLpos = backLeft.getCurrentPosition()-ticks;
        int BRpos = backRight.getCurrentPosition()+ticks;
        setMotorPowers(power1, power2, power3, power4);
        setMotorPositions(FLpos, FRpos, BLpos, BRpos);
        setMode(2);
    }

    public void IntakeClose(){
        claw1.setPosition(0.95);
        claw2.setPosition(0.75);

    }
    public void IntakeOpen() {
        yaw1.setPosition(0.4);
        yaw2.setPosition(0.3);

        claw1.setPosition(0.8);
        claw2.setPosition(1);


    }

    public void raiseLift(int pos) {
        vert.setTargetPosition(pos);
        vert.setPower(0.8);
    }

    public void release(){
        yaw2.setPosition(-0.1);
        claw1.setPosition(0.95);
        claw2.setPosition(0.75);

    }
}
