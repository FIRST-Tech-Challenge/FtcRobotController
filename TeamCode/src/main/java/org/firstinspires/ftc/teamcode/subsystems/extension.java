package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.Range;

public class extension {
    int degreestotickstilt(int degrees){
        return (int)((2786.2/360)*degrees);
    }
    public static int pos=0;
    public PIDFController pidf;
    public static int Liftpos;
    public static int tiltplacepos=0;
    public static int tiltplacelowpos=0;
    public static int tiltplacemidpos=0;
    public static int tiltplacehighpos=0;
    public static int tiltintakeclosepos=0;
    public static int tiltintakefarpos=0;
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

//        ((DcMotorEx) tilt).setPositionPIDFCoefficients(4.75);
        tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //    pidf.setTolerance(5);
  //      pidf.setPIDF(0.004,0,0.005,0.00000001);
        Liftpos = lift1.getCurrentPosition();
        setIntakeClosePos();
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
    public void makemoretilt(){
        if(pos==0){tiltintakeclosepos+=10; setIntakeClosePos();}
        else if(pos==1){tiltintakefarpos+=10; setIntakeFarPos();}
        else if(pos==3){tiltplacelowpos+=10;setPlaceLow();}
        else if(pos==4){tiltplacemidpos+=10;setPlaceMid();}
        else if(pos==5){tiltplacehighpos+=10;setPlaceHigh();}
    }
    public void makelesstilt(){
        if(pos==0){tiltintakeclosepos-=10; setIntakeClosePos();}
        else if(pos==1){tiltintakefarpos-=10; setIntakeFarPos();}
        else if(pos==3){tiltplacelowpos-=10;setPlaceLow();}
        else if(pos==4){tiltplacemidpos-=10;setPlaceMid();}
        else if(pos==5){tiltplacehighpos-=10;setPlaceHigh();}
    }
    public int getTilt(){
        return tilt.getCurrentPosition();
    }
    public void release(){
        tilt.setPower(0);
        lift1.setPower(0);
        lift2.setPower(0);
    }
    public void hold(){tilt.setTargetPosition(tilt.getCurrentPosition()); tilt.setPower(1);}
    //add wrist commands here probably
    //tune all values when stringed and intake is done
    //
    public void setHeight(int ticks){
        lift1.setTargetPosition(ticks);
        lift2.setTargetPosition(ticks);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tilt.setPower(1);
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
        setTilt(tiltintakeclosepos);
        tilt.setPower(1);
        setHeight(0);
        if((tilt.getCurrentPosition())<=155){
        setHeight(0);
        }
    }
    public void setIntakeFarPos(){
        pos=1;
        setTilt(tiltintakefarpos);
        tilt.setPower(1);
        if((tilt.getCurrentPosition())<=155){
            setHeight(0);
        }
    }
    public void setStowPos(){
        pos=2;
        setHeight(0);
        if((lift1.getCurrentPosition())<50) {
            setTilt(0);
        }
        tilt.setPower(Range.clip((Range.scale(Range.clip(tilt.getCurrentPosition(),0,640),0,/*697*/640,1,0)*2),0.1,1));
    }
    public void setPlaceLow(){
            setTilt(tiltplacelowpos);
            tilt.setPower(1);
            pos=3;
            if((tilt.getCurrentPosition())<(tilt.getTargetPosition()*(7.0 / 8.0))){
            setHeight(0);
        }

    }
    public void setPlaceMid(){
        setTilt(tiltplacemidpos);
        tilt.setPower(1);
        pos=4;
        if((tilt.getCurrentPosition())<(tilt.getTargetPosition()*(7.0 / 8.0))){
            setHeight(0);
        }
    }
    public void setPlaceHigh(){
        setTilt(tiltplacehighpos);
        tilt.setPower(1);
        pos=5;
        if((tilt.getCurrentPosition())<(tilt.getTargetPosition()*(7.0 / 8.0))){
            setHeight(0);
        }
    }

    public void setTilt(int ticks){
        tiltpos=ticks;
        tilt.setTargetPosition(tiltpos);
        //if(tilt.getPower()==0)tilt.setPower(1);
        tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void run(){
        if(pos==0)setIntakeClosePos();
        else if(pos==1)setIntakeFarPos();
        else if(pos==2)setStowPos();
        else if(pos==3)setPlaceLow();
        else if(pos==4)setPlaceMid();
        else if(pos==5)setPlaceHigh();
    }

}
