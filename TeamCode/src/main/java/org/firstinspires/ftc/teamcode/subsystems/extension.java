package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDFController;
public class extension {
    int degreestotickstilt(int degrees){
        return (int)((2786.2/360)*degrees);
    }
    public static int pos;
    public PIDFController pidf;
    public static int Liftpos;
    public static int tiltplacepos=675;
    public static int tiltpos;
    public DcMotor lift1;
    public DcMotor tilt;
    public DcMotor lift2;
    public extension(HardwareMap hwmap){
        lift1=hwmap.get(DcMotor.class,"lift1");
        lift2=hwmap.get(DcMotor.class,"lift2");
        tilt=hwmap.dcMotor.get("tilt");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setTargetPosition(0);
        lift2.setTargetPosition(0);
        tilt.setTargetPosition(0);
        lift1.setPower(1);
        lift2.setPower(1);
        tilt.setPower(1);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


      //  ((DcMotorEx) tilt).setVelocityPIDFCoefficients(10, 0, 11, 10);

        ((DcMotorEx) tilt).setPositionPIDFCoefficients(4.75);
        tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //    pidf.setTolerance(5);
  //      pidf.setPIDF(0.004,0,0.005,0.00000001);
        Liftpos = lift1.getCurrentPosition();
    }
    public static boolean laststate=false;
//    public void move_with_hands(boolean yes){
//        if(!yes&&laststate) {
//            tilt.setPower(1);
//            tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            tilt.setTargetPosition(tilt.getCurrentPosition());
//            tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }else if(yes) {
//            tilt.setPower(0);
//            tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//            tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
//        laststate=yes;
//    }
    public void makemoretilt(){tiltplacepos+=10;}
    public void makelesstilt(){tiltplacepos-=10;}
    public int getTilt(){
        return tilt.getCurrentPosition();
    }
    public void release(){
        tilt.setPower(0);
    }
    public void hold(){tilt.setTargetPosition(tilt.getCurrentPosition()); tilt.setPower(1);}
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
    public void sethang() {
        setTilt(510);
    }
    public void setPlace() {setTilt(tiltplacepos);}
    public void setIntakeClosePos(){
        pos=0;
        setTilt(0);
        setHeight(0);
        if((tilt.getCurrentPosition())<(tilt.getTargetPosition()*(7.0 / 8.0))){
        setHeight(0);
        }
    }
    public void setIntakeFarPos(){
        pos=1;
        setTilt(0);
        setHeight(0);
        if((tilt.getCurrentPosition())<(tilt.getTargetPosition()*(7.0 / 8.0))){
            setHeight(0);
        }
    }
    public void setStowPos(){
        pos=2;
        setHeight(0);
        if((lift1.getCurrentPosition())<(lift1.getTargetPosition()*(7.0 / 8.0))){
            setTilt(degreestotickstilt(90));
        }

    }
    public void setPlaceLow(){
        if(pos>1) {
            setTilt(0);
            setHeight(0);
            pos=3;
        }

    }
    public void setPlaceMid(){
        if(pos>1) {
            setTilt(0);
            setHeight(0);
            pos=4;
        }
    }
    public void setPlaceHigh(){
        if(pos>1) {
            setTilt(0);
            setHeight(0);
            pos=4;
        }
    }

    public void setTilt(int ticks){
        tiltpos=ticks;
        tilt.setTargetPosition(tiltpos);
        tilt.setPower(1);
        tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void run(){

    }

}
