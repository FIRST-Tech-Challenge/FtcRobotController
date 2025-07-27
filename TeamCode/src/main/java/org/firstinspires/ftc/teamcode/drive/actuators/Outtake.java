package org.firstinspires.ftc.teamcode.drive.actuators;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Outtake{
    DcMotor poliaright;
    DcMotor polialeft;
    Servo Bright;
    Servo Bleft;
    Servo garrinha;
    double ticks = 2750;
    double ticks2 = 2200;
    double ticks3 = 338;
    double newTarget;
    public Outtake (HardwareMap hardwareMap){
        poliaright = hardwareMap.get(DcMotor.class, "poliaright");
        polialeft = hardwareMap.get(DcMotor.class, "polialeft");
        Bright = hardwareMap.get(Servo.class, "bright");
        Bleft = hardwareMap.get(Servo.class, "bleft");
        garrinha = hardwareMap.get(Servo.class, "garrinha");

        poliaright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        poliaright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        poliaright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        polialeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void slidesDown(){
        poliaright.setTargetPosition(0);
        poliaright.setPower(1);
        poliaright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        polialeft.setTargetPosition(0);
        polialeft.setPower(1);
        polialeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bright.setPosition(1);
        Bleft.setPosition(0);
        sleep(1200);
        polialeft.setPower(0);
        poliaright.setPower(0);
    }
    public void wallClipSet(){
        newTarget = ticks3;
        poliaright.setTargetPosition((int) newTarget);
        poliaright.setPower(1);
        poliaright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        newTarget = ticks3;
        polialeft.setTargetPosition((int) newTarget);
        polialeft.setPower(1);
        polialeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void wranglerClipSet(){
        newTarget = ticks2;
        polialeft.setTargetPosition((int) newTarget);
        polialeft.setPower(1);
        polialeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        newTarget = ticks2;
        poliaright.setTargetPosition((int) newTarget);
        poliaright.setPower(1);
        poliaright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void basketSet(){
        newTarget = ticks;
        poliaright.setTargetPosition((int) newTarget);
        poliaright.setPower(1);
        poliaright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        newTarget = ticks;
        polialeft.setTargetPosition((int) newTarget);
        polialeft.setPower(1);
        polialeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
