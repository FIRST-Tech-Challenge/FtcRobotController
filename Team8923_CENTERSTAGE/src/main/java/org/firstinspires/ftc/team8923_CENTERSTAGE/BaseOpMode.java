package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


abstract public class BaseOpMode extends LinearOpMode {
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    // constants
    static final double TICKS_PER_REVOLUTION = 537.6; // Neverest orbital 20, 7 pulse per revolution, 4 * 7 * 19.2 (the calculations)
    static final double GEAR_RATIO = 1.0;
    static final double WHEEL_DIAMETER = 4.0; // inches
    static final double TICKS_PER_INCH =  (TICKS_PER_REVOLUTION * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);
    static final double INCHES_PER_REVOLUTION = Math.PI * WHEEL_DIAMETER;
    static final double INCHES_PER_TICK = INCHES_PER_REVOLUTION / TICKS_PER_REVOLUTION;
    static final double INTAKE_SPEED = 0.8;

    //imu constants
    public BNO055IMU imu;
    public double startAngle;
    public static final double TURNING_KP = 0.008;
    public static final int ROBOT_HEADING_TOLERANCE_DEGREES = 1;
    public static final double MAXIMUM_TURN_POWER_AUTONOMOUS = 0.7;
    public static final double MINIMUM_TURN_POWER = 0.05;


    public void initHardware() {
        // drive motors
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);

    }

    // mecanum drive method
    public void driveMecanum(double x, double y, double turning) {
        motorFL.setPower(x + y + turning);
        motorFR.setPower(y - x - turning);
        motorBL.setPower(y - x + turning);
        motorBR.setPower(x + y - turning);
    }

}
