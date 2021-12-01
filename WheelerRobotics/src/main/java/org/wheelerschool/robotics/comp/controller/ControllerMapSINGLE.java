package org.wheelerschool.robotics.comp.controller;

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

    }
    private void buttonY(){
        meccanum.spinnySpin(meccanum.HIGH_SPINNER_POWER);
    }
    private void buttonX(){

    }
    private void buttonA(){
        meccanum.spinnySpin(meccanum.OPTIMAL_SPINNER_POWER);
    }
    private void leftTrigger(){
        meccanum.moveArm(-gamepad1.left_trigger);
    }
    private void rightTrigger(){
        meccanum.moveArm(gamepad1.right_trigger);
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
    private void joystickDriver(){
        meccanum.motorDriveXYVectors(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
    }

    public void checkControls(){
        joystickDriver();
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
            leftTrigger();
        }
        else{
            rightTrigger();
        }
    }


}
