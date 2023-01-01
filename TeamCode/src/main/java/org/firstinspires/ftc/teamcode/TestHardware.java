package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by Lance He 9/16/2017. Hello Guys
 */


public class TestHardware {

    // Motor variable names
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;

    public boolean runThisWithEncoder = true;

    // Other variable names
    HardwareMap hwMap;
    public ElapsedTime period = new ElapsedTime();


    public TestHardware() {
        hwMap = null;
        this.runThisWithEncoder = true;
    }


    public TestHardware(boolean runThisWithEncoder) {
        hwMap = null;
        this.runThisWithEncoder = runThisWithEncoder;
    }


    public void initTeleOpIMU(HardwareMap hwMap) {

        // Save reference to Hardware map
        this.hwMap = hwMap;

        period.reset();

        // Define Motors I have changed these below for Mercury

        frontLeftMotor = hwMap.dcMotor.get("front_left");
        frontRightMotor = hwMap.dcMotor.get("front_right");
        backLeftMotor = hwMap.dcMotor.get("back_left");
        backRightMotor = hwMap.dcMotor.get("back_right");

        // these are correct for Mercury

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);


        // May use RUN_USING_ENCODERS if encoders are installed

        if (runThisWithEncoder)
        {
            // Do if encoders are installed
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // get the encoders reset
            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
        else
        {
            // get the encoders reset if it was in that mode previously
            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }



        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // The motor power has a range of -1.0 to 1.0
        // Negative is backwards, Positive is forwards, 0 is off
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);




    }


    public double getTime(){

        return period.time();

    }
    public void sleep(int time){
        period.reset();
        while(period.time() < time / 1000.0){

        }
    }
    public void Go(double speedl, double speedr, int time) {
        frontRightMotor.setPower(speedr);
        frontLeftMotor.setPower(speedl);
        backRightMotor.setPower(speedr);
        backLeftMotor.setPower(speedl);
        sleep(time);
    }
    public void setAllMotors(double powerl, double powerr) {
        frontLeftMotor.setPower(powerl);
        frontRightMotor.setPower(powerr);
        backLeftMotor.setPower(powerl);
        backRightMotor.setPower(powerr);

    }
    public void runUsingEncoders(double power, int newBlPos, int newBrPos, int newFlPos, int newFrPos) {
        //set the new target

        frontLeftMotor.setTargetPosition(newFlPos + frontLeftMotor.getCurrentPosition());
        frontRightMotor.setTargetPosition(newFrPos + frontRightMotor.getCurrentPosition());
        backLeftMotor.setTargetPosition(newBlPos + backLeftMotor.getCurrentPosition());
        backRightMotor.setTargetPosition(newBrPos + backRightMotor.getCurrentPosition());

        //set mode

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power

        setAllMotors(power, power);

        while (frontRightMotor.isBusy()) {

        }
    }
}

