package org.firstinspires.ftc.teamcode.robots.kraken;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Craniumski on 11/10/2017.
 */

public class RelicArm {

    private Servo shoulder;
    private Servo elbow;
    private Servo grip;

    private long extendTimer = 0;
    private float extendDuration = 4.5f;
    public boolean autoExtend = false;
    public boolean autoRetract = false;
    public int placeStage = 0;
    public long placeTimer = 0;

    int shoulderExtend = 2200;
    int shoulderStop = 1500;
    int shoulderRetract = 900;
    int elbowTucked = 1050;
    int elbowApproach = 2125;
    int elbowGrab = 2175;
    int gripOpen = 2125;
    int gripClosed = 825;
    public int elbowMid = (elbowTucked + elbowGrab)/2;

    public int elbowTarget = elbowTucked;

    public RelicArm(Servo shoulder, Servo elbow, Servo grip){
        this.shoulder = shoulder;
        this.elbow = elbow;
        this.grip = grip;
    }



    public boolean autoExtend(){
        if(!autoExtend){
            shoulder.setPosition(servoNormalize(shoulderExtend));
            autoExtend = true;
            extendTimer = futureTime(extendDuration);
        }
        if(extendTimer < System.nanoTime())
        {
            stopShoulder();
            autoExtend = false;
            return true;
        }
        return false;
    }

    public boolean autoRetract(){
        if(!autoExtend){
            shoulder.setPosition(servoNormalize(shoulderRetract));
            autoExtend = true;
            extendTimer = futureTime(extendDuration);
        }
        if(extendTimer < System.nanoTime())
        {
            stopShoulder();
            autoExtend = false;
            return true;
        }
        return false;
    }

    public boolean autoPlace(){
        switch (placeStage){
            case 0:
                elbowTarget = elbowGrab;
                placeTimer = futureTime(1.0f);
                placeStage++;
                break;
            case 1:
                if(placeTimer < System.nanoTime()){
                    openGrip();
                    placeTimer = futureTime(.5f);
                    placeStage++;
                }
            case 2:
                if(placeTimer < System.nanoTime()){
                    retract();
                    placeTimer = futureTime(.1f);
                    placeStage++;
                }
                break;
            case 3:
                if(placeTimer < System.nanoTime()){
                    stopShoulder();
                    placeTimer = futureTime(.2f);
                    placeStage++;
                }
                break;
            case 4:
                if(placeTimer < System.nanoTime()){
                    elbowTarget = elbowTucked;
                    placeTimer = futureTime(1.0f);
                    placeStage++;
                }
                break;
            case 5:
                if(placeTimer < System.nanoTime()){
                    placeStage++;
                }
                break;
            case 6:
                if(autoRetract()){
                    placeStage = 0;
                    return true;
                }
        }
        return false;
    }

    public void extend(){
        autoExtend = false;
        shoulder.setPosition(servoNormalize(shoulderExtend));
    }

    public void stopShoulder(){
        autoExtend = false;
        shoulder.setPosition(servoNormalize(shoulderStop));
    }

    public void retract(){
        autoExtend = false;
        shoulder.setPosition(servoNormalize(shoulderRetract));
    }

    public void deployElbow(){
        if(elbow.getPosition() > servoNormalize(elbowApproach) - .03 && elbow.getPosition() < servoNormalize(elbowApproach) + .03){
//            elbow.setPosition(servoNormalize(elbowGrab));
            elbowTarget = elbowGrab;
        }
        else{
//            elbow.setPosition(servoNormalize(elbowApproach));
            elbowTarget = elbowApproach;
        }
    }

    public void tuckElbow(){
//        elbow.setPosition(servoNormalize(elbowTucked));
        elbowTarget = elbowTucked;
    }

    public void openGrip(){
        grip.setPosition(servoNormalize(gripOpen));
    }

    public void closeGrip(){
        grip.setPosition(servoNormalize(gripClosed));
    }

    public void toggleGrip(){
        if(grip.getPosition() > servoNormalize(gripOpen) - .05 && grip.getPosition() < servoNormalize(gripOpen) + .05){
            closeGrip();
        }
        else{
            openGrip();
        }
    }

    public void setElbow(int pwm){
        elbowTarget = pwm;
    }

    public void setGrip(int pwm){
        grip.setPosition(servoNormalize(pwm));
    }

    public void update(){
        elbow.setPosition(servoNormalize(elbowTarget));
    }



    public static double servoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

    long futureTime(float seconds){
        return System.nanoTime() + (long) (seconds * 1e9);
    }

}
