package org.firstinspires.ftc.team8923_2021;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

abstract public class MasterOpMode extends LinearOpMode {
    //declare drive motors
    public DcMotor motorLeft = null;
    public DcMotor motorRight = null;

    //declare misc motors
    public DcMotor motorIntake = null;
    public DcMotor motorCarousel = null;
    public DcMotor motorLift = null;

    //declare servos
    public Servo servoGrabber = null;

    //declare imu
    public BNO055IMU imu;

    public void initHardware() {
        //init drive motors
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        //reset encoder
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        //init misc motors
        motorIntake = hardwareMap.dcMotor.get("motorIntake");
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorCarousel = hardwareMap.dcMotor.get("motorCarousel");
        motorCarousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLift = hardwareMap.dcMotor.get("motorLift");
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setTargetPosition(motorLift.getCurrentPosition());
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //init servos
        servoGrabber = hardwareMap.get(Servo.class, "servoGrabber");
        servoGrabber.setPosition(0.0);

        //init imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
}