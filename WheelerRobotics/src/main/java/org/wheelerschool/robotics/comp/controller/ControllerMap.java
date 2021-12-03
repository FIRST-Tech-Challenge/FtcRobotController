package org.wheelerschool.robotics.comp.controller;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import static java.lang.Double.max;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.wheelerschool.robotics.comp.chassis.Meccanum;

public class ControllerMap {

    private Meccanum meccanum;
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    public void init(Meccanum mec, Gamepad gp1, Gamepad gp2){
        meccanum = mec;
        gamepad1 = gp1;
        gamepad2 = gp2;

    }

    private void leftBumper(){

    }
    private void rightBumper(){

    }
    private void buttonB(){

    }
    private void buttonY(){
        //meccanum.spinnySpin(meccanum.HIGH_SPINNER_POWER);
    }
    private void buttonX(){

    }
    private void buttonA(){
        //meccanum.spinnySpin(meccanum.OPTIMAL_SPINNER_POWER);
    }
    private void leftTrigger(){
        //meccanum.moveArm(-gamepad1.left_trigger);
    }
    private void rightTrigger(){
        //meccanum.moveArm(gamepad2.right_trigger);
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
        gamepad2.rumble(time);
    }
    private void joystickDriver(){
        meccanum.motorDriveXYVectors(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }

    private void dpadLeft2(){

    }
    private void dpadRight2(){

    }
    private void dpadUp2(){

    }
    private void dpadDown2(){

    }
    private void buttonB2(){

    }
    private void leftBumper2(){
        meccanum.closeServoFull();
    }
    private void rightBumper2(){
        meccanum.openServoFull();
    }
    private void buttonY2(){
        meccanum.spinnySpin(meccanum.HIGH_SPINNER_POWER);
    }

    private void leftTrigger2(double scale){
        meccanum.moveArm(-gamepad1.left_trigger * scale);
    }
    private void rightTrigger2(double scale){
        meccanum.moveArm(gamepad2.right_trigger * scale);
    }

    private void buttonX2(){

    }

    private void buttonA2(){
        meccanum.spinnySpin(meccanum.OPTIMAL_SPINNER_POWER);
    }

    private void rumble2(int time){
        gamepad2.rumble(time);
    }

    private void joystickDriver2(){
        // do some other joystick stuff
    }

    public void checkControls(){
        joystickDriver();
        if (gamepad1.left_bumper){
            leftBumper();
        }
        if (gamepad1.right_bumper){
            rightBumper();
        }
        if (gamepad1.x){
            buttonX();
            rumble2(10);
        }
        if (gamepad1.b){
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
        if (gamepad1.y){
            buttonY();
        }
        if (gamepad1.a){
            buttonA();
        }
        if (gamepad1.left_trigger > gamepad1.right_trigger) {
            leftTrigger();
        }else{
            rightTrigger();
        }
    }
    public void checkControls2(){
        //joystickDriver2();
        if (gamepad2.left_bumper){
            leftBumper2();
        }
        if (gamepad2.right_bumper){
            rightBumper2();
        }
        if (gamepad2.x){
            buttonX2();
        }
        if (gamepad2.b){
            buttonB2();
        }
        if(gamepad2.dpad_down){
            dpadDown2();
        }
        if(gamepad2.dpad_up){
            dpadUp2();
        }
        if(gamepad2.dpad_right){
            dpadRight2();
        }
        if(gamepad2.dpad_left){
            dpadLeft2();
        }
        if (gamepad2.y){
            buttonY2();
        }
        if (gamepad2.a){
            buttonA2();
        }
        if (gamepad2.left_trigger >= gamepad2.right_trigger) {
            leftTrigger2(0.8);
        }
        else {
            rightTrigger2(0.8);
        }
    }


}
