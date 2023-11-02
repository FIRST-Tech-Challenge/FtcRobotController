package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDFController;
public class extension {
    public PIDFController pidf;
    public static int Liftpos;

    public DcMotor lift1;
    public DcMotor tilt;
    public DcMotor lift2;
    public extension(HardwareMap hwmap){
//        lift1=hwmap.get(DcMotor.class,"lift1");
//        lift2=hwmap.get(DcMotor.class,"lift2");
        tilt=hwmap.get(DcMotor.class,"tilt");
//        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//      lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lift1.setTargetPosition(0);
//        lift2.setTargetPosition(0);
        tilt.setTargetPosition(0);
//        lift1.setPower(1);
//        lift2.setPower(1);
        tilt.setPower(1);
//        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        pidf.setTolerance(5);
//        pidf.setPIDF(0.004,0,0.005,0.00000001);
     //   Liftpos = lift1.getCurrentPosition();
    }
    public static boolean laststate=false;
    public void move_with_hands(boolean yes){
        if(!yes&&laststate) {
            tilt.setPower(1);
            tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            tilt.setTargetPosition(tilt.getCurrentPosition());
            tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }else if(yes) {
            tilt.setPower(0);
            tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        laststate=yes;
    }
    //add wrist commands here probably
    //tune all values when stringed and intake is done
    //
    public void setHeight(int ticks){
        lift1.setTargetPosition(ticks);
        lift2.setTargetPosition(ticks);
    }
    public void setIntake() {
        setTilt(0);
    }
    public void setPlace() {
        setTilt(0);
    }
    public void setIntakeClosePos(){

        setTilt(0);
        setHeight(0);
        if((tilt.getCurrentPosition())<(tilt.getTargetPosition()*((double) 3.0 / 4.0))){
        setHeight(0);
        }
    }
    public void setIntakeFarPos(){
        setTilt(0);
        setHeight(0);
        if((tilt.getCurrentPosition())<(tilt.getTargetPosition()*(3.0 / 4.0))){
            setHeight(0);
        }
    }
    public void setStowPos(){
        //setHeight(0);
        //if((lift1.getCurrentPosition())<(lift1.getTargetPosition()*(3.0 / 4.0))){
            setTilt(0);
        //}

    }
    public void setPlaceLow(){
        setTilt(0);
        setHeight(0);

    }
    public void setPlaceMid(){
        setTilt(0);
        setHeight(0);
    }
    public void setPlaceHigh(){
        setTilt(0);
        setHeight(0);
    }

    public void setTilt(int ticks){
        tilt.setTargetPosition(ticks);
    }
    public void run(){

    }

}
