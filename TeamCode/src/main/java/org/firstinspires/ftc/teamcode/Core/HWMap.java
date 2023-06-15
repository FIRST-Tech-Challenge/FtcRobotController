package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class contains all the hardware components that are programmed on our robot and are mapped to the robot as well.
 * Other variables like Telemetry and ElapsedTime are also created.
 */

public class HWMap {
    //Motors
    public DcMotorEx leftBackMotor;
    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightBackMotor;
    public DcMotorEx rightFrontMotor;

    public DcMotorEx linearSlides;

    //IMU
    public BNO055IMU imu;
    public static double imuAngle;

    //Servos
    public Servo gripper;

    public Servo leftServo;
    public Servo rightServo;
    public Servo frontServo;

    //Sensors
    public ColorSensor gripperCS;
    public ColorSensor leftCS;
    public ColorSensor rightCS;
    public ColorSensor middleCS;

    //Other Variables
    HardwareMap hardwareMap;
    public Telemetry telemetry;
    public ElapsedTime timer = new ElapsedTime();

    public HWMap(Telemetry telemetry, HardwareMap hardwareMap){

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        //Mapping Motors
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "LB");
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "LF");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "RB");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RF");

        linearSlides = hardwareMap.get(DcMotorEx.class, "LS");

        //IMU
        imu = this.hardwareMap.get(BNO055IMU.class, "imu");
        initializeIMU();

        //Mapping Servos
        gripper = hardwareMap.get(Servo.class, "gripper");

        leftServo = hardwareMap.get(Servo.class, "LServo");
        rightServo = hardwareMap.get(Servo.class, "RServo");
        frontServo = hardwareMap.get(Servo.class, "FServo");

        //Mapping Sensors
        gripperCS = hardwareMap.get(ColorSensor.class, "gripperCS");
        leftCS = hardwareMap.get(ColorSensor.class, "leftCS");
        rightCS = hardwareMap.get(ColorSensor.class, "rightCS");
        middleCS = hardwareMap.get(ColorSensor.class, "middleCS");


        //Set Motor Direction
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        //Zero Power Behavior
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set Motor Mode
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linearSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double readFromIMU() {
        imuAngle = -imu.getAngularOrientation().firstAngle;
        return imuAngle;
    }

    public void initializeIMU() {
        // don't touch please
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }
}
