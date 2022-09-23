package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;

/***
 *  one motor - to rotate turntable
 *  4 Positions - Top, Right, Back, Left
 */

public class TurnTableSystem {
    // TODO

    public final DcMotor rotatorMotor;

    public static final int LEVEL_90 = 4209; // to test
    public static final int LEVEL_0 = 0; // to test

    public int getPosition(){
        return rotatorMotor.getCurrentPosition();
    }

//    public void moveToPosition(int state){
//        rotatorMotor.setTargetPosition(state);
//        rotatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); //is this right? Refer to last year's YeetSystem
//        if(state > rotatorMotor.getCurrentPosition()){
//            rotatorMotor.setPower(0.75);
//        }
//        else{
//            rotatorMotor.setPower(-0.75);
//        }
//    }

    public boolean moveToPosition(int ticks){
        rotatorMotor.setTargetPosition(ticks);
        rotatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotatorMotor.setPower(0.75);
        return getPosition() == ticks;
    }

    public void resetPosition() {
        rotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void move_right() {
        rotatorMotor.setTargetPosition(rotatorMotor.getCurrentPosition() - 80);
        rotatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotatorMotor.setPower(1.0);
    }

    /**
     * Intakes rings
     */
    public void move_left() {
        rotatorMotor.setTargetPosition(rotatorMotor.getCurrentPosition() + 80);
        rotatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotatorMotor.setPower(-1.0);

    }
    public void stop() {
        rotatorMotor.setPower(0.0);
    }

    //TODO - TEST + CHANGE THESE NUMBERS
    private static final double CLOSED_POSITION = 0.3;
    private static final double OPEN_POSITION = 0.93;

    private Map<CircleState, Integer> mapToPosition;

    private enum CircleState {
        LEVEL_FORWARD,
        LEVEL_RIGHT,
        LEVEL_BACK,
        LEVEL_LEFT,
    }

    private CircleState currentArmState;

    private void initMotors() {
        rotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rotatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rotatorMotor.setPower(0);
    }

    public TurnTableSystem(DcMotor rotatorMotor){
        this.rotatorMotor = rotatorMotor;
        mapToPosition = new HashMap<>();
        mapToPosition.put(CircleState.LEVEL_BACK, 32); // TODO
        mapToPosition.put(CircleState.LEVEL_FORWARD, 16);
        mapToPosition.put(CircleState.LEVEL_RIGHT, 8);
        mapToPosition.put(CircleState.LEVEL_LEFT, 4);
        initMotors();
    }

    private void goToSetPosition(CircleState state){
        rotatorMotor.setPower(0);
        rotatorMotor.setTargetPosition((this.mapToPosition.get(state)));
        rotatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); //is this right? Refer to last year's YeetSystem
        rotatorMotor.setPower(0.75);
        //rotatorMotor.setPower(0);
    }

    public void moveCounter(){
        rotatorMotor.setPower(0.5);
    }

    public void moveClock(){
        rotatorMotor.setPower(-0.5);
    }

    private void goToPosition(int pos){
        rotatorMotor.setPower(0);
        rotatorMotor.setTargetPosition(pos);
        rotatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); //is this right? Refer to last year's YeetSystem
        rotatorMotor.setPower(0.75);
        //rotatorMotor.setPower(0);
    }

    private void place(int level, CircleState state){
        /*if (bool){
            releaser.setPosition(CLOSED_POSITION);
        }
        else{
            releaser.setPosition(OPEN_POSITION);
        }
        long initTime = System.currentTimeMillis();
        boolean enoughTimeElapsed = false;
        while (enoughTimeElapsed){ */
            /*if (System.currentTimeMillis() - initTime > 0.55){ */ /* TODO: FIND TIME TO DELAY/GO DOWN
                enoughTimeElapsed = true;
            }
            else{

            }
        }
        releaser.setPosition(CLOSED_POSITION);*/
    }

    /*public boolean arm(){
        switch (currentArmState){
            case LEVEL_TOP:
                break;
            case LEVEL_MID:
                break;
            case LEVEL_BOTTOM:
                break;
        }
        return false;
    }*/
}