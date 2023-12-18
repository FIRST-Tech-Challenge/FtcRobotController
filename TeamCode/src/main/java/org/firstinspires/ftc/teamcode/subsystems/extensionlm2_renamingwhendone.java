package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionStates;

public class extensionlm2_renamingwhendone {

    int degreestotickstilt(int degrees){
        return (int)((2786.2/360)*degrees);
    }
    public static int pos=0;
    public PIDFController pidf;
    public static int Liftpos;
    public static int tiltplacepos=0;
    public static int tiltplacelowpos=-687;
    public static int tiltplacemidpos=-687;
    public static int tiltplacehighpos=-687;
    public static int tiltintakeclosepos=-40;
    public static int tiltintakefarpos=-30;
    public static int tiltpos;
    public DcMotor lift1;
    public DcMotor tilt;
    public DcMotor lift2;
    ExtensionStates state= ExtensionStates.STOW;
    public void init(HardwareMap hwmap){
//        lift1=hwmap.get(DcMotor.class,"lift1");
//        lift2=hwmap.get(DcMotor.class,"lift2");
        tilt=hwmap.dcMotor.get("tilt");
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
//        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        ((DcMotorEx) tilt).setPositionPIDFCoefficients(5.5);
        tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setStowPos();
    }
    public static boolean laststate=false;

    public void makelesstilt(){
        if(state==ExtensionStates.INTAKE_CLOSE){tiltintakeclosepos+=5; setIntakeClosePos();}
        else if(state==ExtensionStates.INTAKE_FAR){tiltintakefarpos+=5; setIntakeFarPos();}
        else if(state==ExtensionStates.STOW){tiltintakefarpos+=5; setStowPos();}
        else if(state==ExtensionStates.PLACE_LOW){tiltplacelowpos+=5;setPlaceLow();}
        else if(state==ExtensionStates.PLACE_MID){tiltplacemidpos+=5;setPlaceMid();}
        else if(state==ExtensionStates.PLACE_HIGH){tiltplacehighpos+=5;setPlaceHigh();}
    }
    public void makemoretilt(){
        if(state==ExtensionStates.INTAKE_CLOSE){tiltintakeclosepos-=5; setIntakeClosePos();}
        else if(state==ExtensionStates.INTAKE_FAR){tiltintakefarpos-=5; setIntakeFarPos();}
        else if(state==ExtensionStates.STOW){tiltintakefarpos-=5; setStowPos();}
        else if(state==ExtensionStates.PLACE_LOW){tiltplacelowpos-=5;setPlaceLow();}
        else if(state==ExtensionStates.PLACE_MID){tiltplacemidpos-=5;setPlaceMid();}
        else if(state==ExtensionStates.PLACE_HIGH){tiltplacehighpos-=5;setPlaceHigh();}
    }
    public int getTilt(){
        return tilt.getCurrentPosition();
    }
    public void release(){
        tilt.setPower(0);
      //  lift1.setPower(0);
        //lift2.setPower(0);
    }

    //add wrist commands here probably

    public void setHeight(int ticks){
        lift1.setTargetPosition(ticks);
        lift2.setTargetPosition(ticks);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void setTilt(int ticks){
        tiltpos=ticks;
        tilt.setTargetPosition(tiltpos);
        //if(tilt.getPower()==0)tilt.setPower(1);
        tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setIntakeClosePos(){
        if(!(state==ExtensionStates.INTAKE_FAR||state==ExtensionStates.INTAKE_CLOSE))setStowPos();
        state=ExtensionStates.INTAKE_CLOSE;
        setTilt(tiltintakeclosepos);
        tilt.setPower(1);
       // setHeight(0);

    }
    public void setIntakeFarPos(){
        if(!(state==ExtensionStates.INTAKE_FAR||state==ExtensionStates.INTAKE_CLOSE))setStowPos();
        state=ExtensionStates.INTAKE_FAR;
        setTilt(tiltintakefarpos);
        tilt.setPower(1);
        //if((tilt.getCurrentPosition())<=155){
        //    setHeight(0);
        //}
    }
    public void setStowPos(){
        state=ExtensionStates.STOW;
        boolean toohigh= !(state==ExtensionStates.INTAKE_FAR||state==ExtensionStates.INTAKE_CLOSE||state==ExtensionStates.STOW);
       // setHeight(0);
        //if((lift1.getCurrentPosition())<50) {
            setTilt(-50);
      //  }
        if(toohigh)tilt.setPower(Range.clip((Range.scale(Range.clip(Math.abs(tilt.getTargetPosition()-tilt.getCurrentPosition()),0,600),0,600,0,1)*2),0.1,1));
        if(!toohigh)tilt.setPower(1);
    }
    public void setPlaceLow(){
        state=ExtensionStates.PLACE_LOW;
            setTilt(tiltplacelowpos);
            tilt.setPower(1);
//            if((tilt.getCurrentPosition())<(tilt.getTargetPosition()*(7.0 / 8.0))){
//            setHeight(0);
//        }

    }
    public void setPlaceMid(){
        state=ExtensionStates.PLACE_MID;
        setTilt(tiltplacemidpos);
        tilt.setPower(1);
//        if((tilt.getCurrentPosition())<(tilt.getTargetPosition()*(7.0 / 8.0))){
//            setHeight(0);
//        }
    }
    public void setPlaceHigh(){
        state=ExtensionStates.PLACE_HIGH;
        setTilt(tiltplacehighpos);
        tilt.setPower(1);
//        if((tilt.getCurrentPosition())<(tilt.getTargetPosition()*(7.0 / 8.0))){
//            setHeight(0);
//        }
    }


    public void run(){
        if(state==ExtensionStates.INTAKE_CLOSE)setIntakeClosePos();
        else if(state==ExtensionStates.INTAKE_FAR)setIntakeFarPos();
        else if(state==ExtensionStates.STOW)setStowPos();
        else if(state==ExtensionStates.PLACE_LOW)setPlaceLow();
        else if(state==ExtensionStates.PLACE_MID)setPlaceMid();
        else if(state==ExtensionStates.PLACE_HIGH)setPlaceHigh();
    }

}
