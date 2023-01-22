package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware2 {

    // Motor variable names
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;
    public DcMotor verticalLiftMotor = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;
    public BNO055IMU imu;

    public boolean runThisWithEncoder = true;


    // Other variable names
    HardwareMap hwMap;
    private ElapsedTime period = new ElapsedTime();


    public Hardware2() {
        hwMap = null;
        this.runThisWithEncoder = true;
    }


    public Hardware2(boolean runThisWithEncoder) {
        hwMap = null;
        this.runThisWithEncoder = runThisWithEncoder;
    }


    public void initTeleOpIMU(HardwareMap hwMap) {

        // Save reference to Hardware map
        this.hwMap = hwMap;

        period.reset();



        frontLeftMotor = hwMap.dcMotor.get("front_left");
        frontRightMotor = hwMap.dcMotor.get("front_right");
        backLeftMotor = hwMap.dcMotor.get("back_left");
        backRightMotor = hwMap.dcMotor.get("back_right");
        verticalLiftMotor = hwMap.dcMotor.get("vertical_lift");
        leftClaw = hwMap.servo.get("leftClaw");
        rightClaw = hwMap.servo.get("rightClaw");


        // Initialize Motors


        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        verticalLiftMotor.setDirection(DcMotor.Direction.FORWARD);

        // May use RUN_USING_ENCODER if encoders are installed

        if (runThisWithEncoder)
        {
            // Do if encoders are installed and you want to use them
            startEncoderMode();
        }
        else
        {
            stopEncoderMode();
        }


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // The motor power has a range of -1.0 to 1.0
        // Negative is backwards, Positive is forwards, 0 is off
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES; //unit for turning
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        imu.initialize(parameters);



    }


    public void startEncoderMode()
    {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get the encoders reset
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void stopEncoderMode()
    {
        // get the encoders reset if it was in that mode previously
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void setPowerOfAllMotorsTo(double power)
    {
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
    }


    public double getTime(){

        return period.time();

    }
    public DcMotor getBackLeftDrive() {
        return backLeftMotor;
    }
    public DcMotor getLeftDrive() {
        return frontLeftMotor;
    }
    public DcMotor getBackRightDrive() {
        return backRightMotor;
    }
    public DcMotor getRightDrive() {
        return frontRightMotor;
    }
    public Servo getLeftClaw() {
        return leftClaw;
    }

    public Servo getRightClaw() {
        return rightClaw;
    }

    public DcMotor getArm() { return verticalLiftMotor; }
    public BNO055IMU getImu() {return imu;}
}


