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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import java.util.ArrayList;
import java.util.Arrays;

public class HardwareMapV2 {
    public DcMotor frontRight = null, frontLeft = null, backRight = null, backLeft = null, intake = null, outtake = null;

    public Servo leftTilt = null, rightTilt = null, wobble = null, slapper = null;
    boolean odometry = false;
    boolean odometryTest = false;
    boolean initIMU;

    VoltageSensor voltageSensor;


    ArrayList<DcMotor> motors = new ArrayList<>(Arrays.asList(frontRight, frontLeft, backLeft, backRight, intake, outtake));
    ArrayList<DcMotor> odomotors = new ArrayList<>(Arrays.asList(frontRight, frontLeft));
    ArrayList<? extends HardwareDevice> servos = new ArrayList<>(Arrays.asList(slapper, leftTilt, rightTilt, wobble));

    ModernRoboticsI2cGyro realgyro1;
    HardwareMap hwMap = null;

    BNO055IMU               imu;


    public HardwareMapV2 (boolean initIMU){
        this.initIMU = initIMU;
    }

    public void init (HardwareMap awhMap) {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        hwMap = awhMap;
        imu = hwMap.get(BNO055IMU.class, "imu");


        frontLeft = hwMap.get(DcMotor.class, "front_left");
        frontRight = hwMap.get(DcMotor.class, "front_right");
        backRight = hwMap.get(DcMotor.class, "back_right");
        backLeft = hwMap.get(DcMotor.class, "back_left");
        intake = hwMap.get(DcMotor.class, "succ");
        outtake = hwMap.get(DcMotor.class, "spit");

//        conveyor = hwMap.get(CRServo.class, "convey");
        slapper = hwMap.get(Servo.class, "slappa");
        leftTilt = hwMap.get(Servo.class, "left_tilt");
        rightTilt = hwMap.get(Servo.class, "right_tilt");
        wobble = hwMap.get(Servo.class, "wobble");

        frontLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        outtake.setDirection(DcMotor.Direction.REVERSE);

        if (odometry) {

        }
        if (odometryTest){
            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        leftTilt.setDirection(Servo.Direction.REVERSE);
        rightTilt.setDirection(Servo.Direction.FORWARD);
        wobble.setDirection(Servo.Direction.FORWARD);
        slapper.setDirection(Servo.Direction.FORWARD);

        slapper.setPosition(0.0);
        pause(1000);
        rightTilt.setPosition(0.5);
        leftTilt.setPosition(0.5);
        wobble.setPosition(0.0);

        if (initIMU){
            imu.initialize(parameters);
        }
        voltageSensor = hwMap.voltageSensor.iterator().next();

    }
    public void pause(double milis){
        double time = System.currentTimeMillis() + milis;
        while (time >= System.currentTimeMillis()) {}
    }


    public void setEncoders(ArrayList<DcMotor> motors, DcMotor.RunMode... modes){
        for (DcMotor motor: motors){
            for (DcMotor.RunMode mode: modes)
                motor.setMode(mode);
        }
    }

    public void setEncoders(DcMotor.RunMode... mode){
        setEncoders(motors, mode);
        setEncoders(odomotors, mode);
    }

    public void setPowerAll(double power){
        for (DcMotor motor: motors){
            motor.setPower(power);
        }
    }

    public void setMotorEncoders(){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getVoltage(){
        return voltageSensor.getVoltage();
    }

}
