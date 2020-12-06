package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.Arrays;

public class HardwareMapV2 {
    public DcMotor frontRight = null, frontLeft = null, backRight = null, backLeft = null, intake = null, outtake = null;
    public DcMotor leftVertical = null, rightVertical = null, horizontal = null;

    public CRServo conveyor = null;
    public Servo leftTilt = null, rightTilt = null;
    boolean odometry = false;

    ArrayList<DcMotor> motors = new ArrayList<>(Arrays.asList(frontRight, frontLeft, backLeft, backRight, intake, outtake));
    ArrayList<DcMotor> odomotors = new ArrayList<>(Arrays.asList(leftVertical, rightVertical, horizontal));
    ArrayList<? extends HardwareDevice> servos = new ArrayList<>(Arrays.asList(conveyor, leftTilt, rightTilt));

    ModernRoboticsI2cGyro realgyro1;
    HardwareMap hwMap = null;

    public HardwareMapV2 (){

    }

    public void init (HardwareMap awhMap) {

        hwMap = awhMap;
        frontLeft = hwMap.get(DcMotor.class, "front_left");
        frontRight = hwMap.get(DcMotor.class, "front_right");
        backRight = hwMap.get(DcMotor.class, "back_right");
        backLeft = hwMap.get(DcMotor.class, "back_left");
        intake = hwMap.get(DcMotor.class, "succ");
        outtake = hwMap.get(DcMotor.class, "spit");

        if (odometry) {
            leftVertical = hwMap.dcMotor.get("left_vertical");
            rightVertical = hwMap.dcMotor.get("right_vertical");
            horizontal = hwMap.dcMotor.get("horizontal");
        }
        conveyor = hwMap.get(CRServo.class, "convey");
        leftTilt = hwMap.get(Servo.class, "left_tilt");
        rightTilt = hwMap.get(Servo.class, "right_tilt");

        frontLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        outtake.setDirection(DcMotor.Direction.FORWARD);

        if (odometry) {
            leftVertical.setDirection(DcMotorSimple.Direction.REVERSE);
            rightVertical.setDirection(DcMotorSimple.Direction.FORWARD);
            horizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        //conveyor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftTilt.setDirection(Servo.Direction.REVERSE);
        rightTilt.setDirection(Servo.Direction.FORWARD);
        conveyor.setDirection(CRServo.Direction.FORWARD);

    }

    public void setEncoders(ArrayList<DcMotor> motors, DcMotor.RunMode mode){
        for (DcMotor motor: motors){
            motor.setMode(mode);
        }
    }

    public void setEncoders(DcMotor.RunMode mode){
        setEncoders(motors, mode);
        setEncoders(odomotors, mode);
    }

    public void setPowerAll(double power){
        for (DcMotor motor: motors){
            motor.setPower(power);
        }
    }

}
