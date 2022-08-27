package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class hardware extends LinearOpMode {
    static final int MOTOR_TICK_COUNTS=288;
    public DcMotor  frontLeft   = null;
    public DcMotor  frontRight  = null;
    public DcMotor  backLeft    = null;
    public DcMotor  backRight   = null;

    public RevColorSensorV3 sensorColor=null;


    LinearOpMode hw  =  null;
    public hardware() {

    }

    @Override
    public void runOpMode() throws InterruptedException {

    }

    //init function
    public void init(LinearOpMode ahwMap){
        hw= ahwMap;

        // Define and Initialize Motors
        frontLeft  = hw.hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hw.hardwareMap.get(DcMotor.class, "frontRight");
        backLeft    = hw.hardwareMap.get(DcMotor.class, "backLeft");
        backRight   = hw.hardwareMap.get(DcMotor.class, "backRight");
        //right motors are forward by default
//        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//        frontRight.setDirection(DcMotor.Direction.REVERSE);
//        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sensorColor=hw.hardwareMap.get(RevColorSensorV3.class, "sensorColor");


    }
    public void moveRobot(){
        //frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setPower(0.5);
        sleep(1000);
        frontLeft.setPower(0);

        //frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setPower(0.5);
        sleep(1000);
        frontRight.setPower(0);

        //backLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setPower(0.5);
        sleep(1000);
        backLeft.setPower(0);

        //backRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setPower(0.5);
        sleep(1000);
        backRight.setPower(0);

    }
    //moveForward function
    public void moveForward(){




        /*frontLeft.setTargetPosition(encoderDrivingTarget);
        frontRight.setTargetPosition(encoderDrivingTarget);
        backLeft.setTargetPosition(encoderDrivingTarget);
        backRight.setTargetPosition(encoderDrivingTarget);*/
            frontLeft.setPower(-0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(-0.5);
            hw.telemetry.addData("moving",true);
            hw.telemetry.update();
while (getSensorDistance()>1){

        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);


    }
    //moveBackward function
    public void moveBackward(double inches){
        double circumference=3.14*4;
        double rotationsNeeded=inches/circumference;
        int encoderDrivingTarget=(int)(rotationsNeeded*MOTOR_TICK_COUNTS);

        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(0);
        backLeft.setTargetPosition(0);
        backRight.setTargetPosition(0);

        frontLeft.setPower(0.5);
        frontRight.setPower(0.5);
        backLeft.setPower(0.5);
        backRight.setPower(0.5);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()){
            //do nothing until robot finishes driving certain distance
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public double getSensorDistance(){
        double val=sensorColor.getDistance(DistanceUnit.INCH);
        return val;
    }

    public boolean isBlue(){
        boolean blue = false;
        sensorColor.enableLed(true);
        if(sensorColor.blue()>sensorColor.red()){
            blue=true;
        }
        else if(sensorColor.red()>sensorColor.blue()){
            blue=false;
        }
        return blue;
    }

    public double getRobotDistance(){
        int ticks=frontLeft.getCurrentPosition();
        double circ=3.14*4;
        double inches=circ*(ticks/288)+getSensorDistance();
        return inches;


    }


}


