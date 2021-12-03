package org.wheelerschool.robotics.comp.controller;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import static java.lang.Double.max;
import static java.lang.Double.min;

import android.content.Context;
import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.wheelerschool.robotics.comp.chassis.Meccanum;

public class ControllerMapSINGLE {

    private Meccanum meccanum;
    private Gamepad gamepad1;


    public void init(Meccanum mec, Gamepad gamepad){
        meccanum = mec;
        gamepad1 = gamepad;
    }

    private void leftBumper(){
        meccanum.closeServoFull();
    }
    private void rightBumper(){
        meccanum.openServoFull();
    }
    private void buttonB(){
        int startupID;
        Context appContext;
        startupID = hardwareMap.appContext.getResources().getIdentifier("startup", "raw", hardwareMap.appContext.getPackageName());
        appContext = hardwareMap.appContext;
        SoundPlayer.getInstance().startPlaying(appContext, startupID);
    }
    private void buttonY(){
        meccanum.spinnySpin(meccanum.HIGH_SPINNER_POWER);
    }
    private void buttonX(){

    }
    private void buttonA(){
        meccanum.spinnySpin(meccanum.OPTIMAL_SPINNER_POWER);
    }
    private void leftTrigger(double scale){
        meccanum.moveArm(-gamepad1.left_trigger * scale);
    }
    private void rightTrigger(double scale){
        meccanum.moveArm(gamepad1.right_trigger * scale);
    }
    private void dpadLeft(){

    }
    private void dpadRight(){

    }
    private void dpadUp(){

    }
    private void dpadDown(){

    }
    private void rumble(int time){
        gamepad1.rumble(time);
    }
    private void joystickDriver(double scale){
        meccanum.motorDriveXYVectors(gamepad1.left_stick_x * scale, gamepad1.left_stick_y * scale, gamepad1.right_stick_x * 0.6);
    }

    public void checkControls(){
        joystickDriver(0.8);
        if (gamepad1.left_bumper){
            leftBumper();
        }
        if (gamepad1.right_bumper){
            rightBumper();
        }

        // arm position button maps (for later)
        if (gamepad1.x){
            // arm set angle up
            // problem with this is that the arm needs to start at a standard start position
            buttonX();
        }
        if (gamepad1.b){
            // arm set angle down
            // problem with this is that the arm needs to start at a standard start position
            buttonB();
        }
        if(gamepad1.dpad_down){
            dpadDown();
        }
        if(gamepad1.dpad_up){
            dpadUp();
        }
        if(gamepad1.dpad_right){
            dpadRight();
        }
        if(gamepad1.dpad_left){
            dpadLeft();
        }


        //spinner  button maps
        if (gamepad1.y){
            buttonY();
        }else if (gamepad1.a){
            buttonA();
        }else {
            meccanum.spinnyStop();
        }

        //claw arm button map (test adaptive triggers)


        if (gamepad1.left_trigger >= gamepad1.right_trigger) {
            leftTrigger(0.5);
        }
        else{
            rightTrigger(0.5);
        }
    }


}
