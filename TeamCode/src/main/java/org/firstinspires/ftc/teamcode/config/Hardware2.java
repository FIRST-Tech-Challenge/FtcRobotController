package org.firstinspires.ftc.teamcode.config;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware2 extends OpMode {

    // Motor variable names
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;
    public DcMotor verticalLiftMotor = null;
    public CRServo intakeServo = null;

    public boolean runThisWithEncoder = true;
    public BNO055IMU imu;

    // Other variable names
    HardwareMap hwMap;
    private ElapsedTime period = new ElapsedTime();


    public Hardware2() {
        this.runThisWithEncoder = true;
    }

    @Override
    public void init() {


    }

    @Override
    public void loop() {

    }


    public Hardware2(boolean runThisWithEncoder) {
        this.runThisWithEncoder = runThisWithEncoder;
    }


    public void initTeleOpIMU() {

//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//        parameters.mode = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled = false;
//        imu.initialize(parameters);

        // Save reference to Hardware map
        this.hwMap = hardwareMap;

        period.reset();


        frontLeftMotor = this.hwMap.dcMotor.get("front_left");
        frontRightMotor = this.hwMap.dcMotor.get("front_right");
        backLeftMotor = this.hwMap.dcMotor.get("back_left");
        backRightMotor = this.hwMap.dcMotor.get("back_right");
        verticalLiftMotor = this.hwMap.dcMotor.get("vertical_lift");
        intakeServo = this.hwMap.crservo.get("intake_servo");


        // Initialize Motors


        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        verticalLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeServo.resetDeviceConfigurationForOpMode();




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
        verticalLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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


}