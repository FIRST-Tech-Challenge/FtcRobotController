package org.firstinspires.ftc.teamcode.TestBot;
//Test
//By Ethan Clawsie and Aman Sulaiman, 2021-2022 Freight Frenzy

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;


public class TestHardware {
    public DcMotor frontLeft, backLeft, frontRight, backRight, cascadeMotorRight, cascadeMotorLeft, arm;
    public TouchSensor touchRight, touchLeft;
    public ColorSensor colorSensor;
    public Servo claw;
    public CRServo wrist;
    public DistanceSensor distanceSensor;
    HardwareMap hwMap;
    static final double COUNTS_PER_MOTOR_REV = 384.5;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public ElapsedTime timer = new ElapsedTime();



    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        timer.reset();
        backLeft = hwMap.dcMotor.get("lb");
        backRight = hwMap.dcMotor.get("rb");
        frontLeft = hwMap.dcMotor.get("lf");
        frontRight = hwMap.dcMotor.get("rf");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void setPowerOfAllMotorsTo(double power) {
        backLeft.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        frontRight.setPower(power);
    }
    public void turnOnEncoders(){
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
    public void turnOffEncoders(){
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void encoderDrive(double inches){
        turnOnEncoders();

        backLeft.setTargetPosition((int)(-inches * COUNTS_PER_INCH));
        backRight.setTargetPosition((int)(-inches * COUNTS_PER_INCH));
        frontLeft.setTargetPosition((int)(-inches * COUNTS_PER_INCH));
        frontRight.setTargetPosition((int)(-inches * COUNTS_PER_INCH));

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (backLeft.getTargetPosition() - backLeft.getCurrentPosition() != 0) {
            setPowerOfAllMotorsTo(.8);
        }
        setPowerOfAllMotorsTo(0);

        resetEncoders();

    }
    public void encoderStrafe(double inches){
        turnOnEncoders();

        backLeft.setTargetPosition(-(int)(inches * COUNTS_PER_INCH));
        backRight.setTargetPosition((int)(inches * COUNTS_PER_INCH));
        frontLeft.setTargetPosition((int)(inches * COUNTS_PER_INCH));
        frontRight.setTargetPosition(-(int)(inches * COUNTS_PER_INCH));

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (backLeft.getTargetPosition() - backLeft.getCurrentPosition() != 0) {
            setPowerOfAllMotorsTo(.8);
        }
        setPowerOfAllMotorsTo(0);
        resetEncoders();

    }
    public void encoderTurn(double inches){
        turnOnEncoders();

        backLeft.setTargetPosition(-(int)(inches * COUNTS_PER_INCH));
        backRight.setTargetPosition(-(int)(inches * COUNTS_PER_INCH));
        frontLeft.setTargetPosition((int)(inches * COUNTS_PER_INCH));
        frontRight.setTargetPosition((int)(inches * COUNTS_PER_INCH));

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (backLeft.getTargetPosition() - backLeft.getCurrentPosition() != 0) {
            setPowerOfAllMotorsTo(.8);
        }
        setPowerOfAllMotorsTo(0);

        resetEncoders();
    }
    public void resetEncoders(){
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}


//650, -17, -650, -2, 1277, 609, -14,
// 653, -8, -652, 647, 1280,
// 646, -3,