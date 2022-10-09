package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Robots.PwPRobot;

import java.util.ArrayList;


public class Lift {
    public enum LiftStates {
        LIFT_GROUND(true, "LIFT_GROUND"),
        LIFT_GROUND_RAISING(false, "LIFT_GROUND_RAISING"),
        LIFT_GROUND_JUNCTION(false, "LIFT_GROUND_JUNCTION"),
        LIFT_GROUND_JUNCTION_RAISING(false, "LIFT_GROUND_JUNCTION_RAISING"),
        LIFT_GROUND_JUNCTION_LOWERING(false, "LIFT_GROUND_JUNCTION_LOWERING"),
        LIFT_LOW(false, "LIFT_LOW"),
        LIFT_LOW_RAISING(false, "LIFT_LOW_RAISING"),
        LIFT_LOW_LOWERING(false, "LIFT_LOW_LOWERING"),
        LIFT_MID(false, "LIFT_MID"),
        LIFT_MID_RAISING(false, "LIFT_MID_RAISING"),
        LIFT_MID_LOWERING(false, "LIFT_MID_LOWERING"),
        LIFT_HIGH(false, "LIFT_HIGH"),
        LIFT_HIGH_LOWERING(false, "LIFT_HIGH_LOWERING");

        boolean status;
        String name;

        LiftStates(boolean value, String name) {
            this.status = value;
            this.name = name;
        }

        public void setStatus(boolean status) {
            this.status = status;
            if(status==true){
                for(int i = 0; i< ClawExtension.ClawExtensionStates.values().length; i++){
                    if(this.name != ClawExtension.ClawExtensionStates.values()[i].name()){
                        ClawExtension.ClawExtensionStates.values()[i].setStatus(false);
                    }
                }
            }
        }
    }
    public enum LiftConstants{
        LIFT_HIGH_JUNCTION(200),
        LIFT_MED_JUNCTION(130),
        LIFT_LOW_JUNCTION(70),
        LIFT_GROUND_JUNCTION(0.0);

        double value;
        public double getValue(){
            return value;
        }
        LiftConstants(double num_of_ticks) {
            value = num_of_ticks;
        }
        //make enum for all the tick counts for ground low med high junctions, can set with setGoal(int goal);
    }
    //TODO: RFMotor
    private RFMotor liftMotor;
    private double MAX_LIFT_TICKS = 10000, liftTarget = 0;
    public Lift(){ //constructor
        // hardware map
        //logger.createFile("LiftLog", "Time Junction Ticks");
        liftMotor = new RFMotor("liftMotor", DcMotorEx.RunMode.RUN_USING_ENCODER, true, MAX_LIFT_TICKS, 0);
        liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    public int getLiftPosition(){
        return liftMotor.getCurrentPosition();
    }
    public double getLiftVelocity(){
        return liftMotor.getVelocity();
    }
    private void updateLiftStates(){
        int liftPos = liftMotor.getCurrentPosition();
        double liftVelocity = liftMotor.getVelocity();
        if(liftPos < 10 && liftVelocity == 0){
            LiftStates.LIFT_GROUND.setStatus(true);
        }
        if(liftPos > 0 && Math.abs(liftPos - LiftConstants.LIFT_GROUND_JUNCTION.value) > 10 && liftVelocity > 0){
            LiftStates.LIFT_GROUND_RAISING.setStatus(true);
        }
        if(liftPos > 0 && Math.abs(liftPos - LiftConstants.LIFT_GROUND_JUNCTION.value) > 10 && liftVelocity < 0){
            LiftStates.LIFT_GROUND_JUNCTION_LOWERING.setStatus(true);
        }
        if(Math.abs(LiftConstants.LIFT_GROUND_JUNCTION.value - liftPos) < 10 && liftVelocity == 0){
            LiftStates.LIFT_GROUND_JUNCTION.setStatus(true);
        }
        if(liftPos > LiftConstants.LIFT_GROUND_JUNCTION.value && liftPos < LiftConstants.LIFT_LOW_JUNCTION.value && Math.abs(liftPos - LiftConstants.LIFT_LOW_JUNCTION.value) > 10 && liftVelocity > 0){
            LiftStates.LIFT_GROUND_JUNCTION_RAISING.setStatus(true);
        }
        if(liftPos > LiftConstants.LIFT_GROUND_JUNCTION.value && liftPos < LiftConstants.LIFT_LOW_JUNCTION.value && Math.abs(liftPos - LiftConstants.LIFT_LOW_JUNCTION.value) > 10 && liftVelocity > 0){
            LiftStates.LIFT_LOW_LOWERING.setStatus(true);
        }
        if(Math.abs(LiftConstants.LIFT_LOW_JUNCTION.value - liftPos) < 10 && liftVelocity == 0){
            LiftStates.LIFT_LOW.setStatus(true);
        }
        if(liftPos > LiftConstants.LIFT_LOW_JUNCTION.value && liftPos < LiftConstants.LIFT_MED_JUNCTION.value && Math.abs(liftPos - LiftConstants.LIFT_MED_JUNCTION.value) > 10 && liftVelocity > 0){
            LiftStates.LIFT_LOW_RAISING.setStatus(true);
        }
        if(liftPos > LiftConstants.LIFT_LOW_JUNCTION.value && liftPos < LiftConstants.LIFT_MED_JUNCTION.value && Math.abs(liftPos - LiftConstants.LIFT_MED_JUNCTION.value) > 10 && liftVelocity < 0){
            LiftStates.LIFT_MID_LOWERING.setStatus(true);
        }
        if(Math.abs(LiftConstants.LIFT_MED_JUNCTION.value - liftPos) < 10 && liftVelocity == 0){
            LiftStates.LIFT_MID.setStatus(true);
        }
        if(liftPos > LiftConstants.LIFT_MED_JUNCTION.value && liftPos < LiftConstants.LIFT_HIGH_JUNCTION.value && Math.abs(liftPos - LiftConstants.LIFT_HIGH_JUNCTION.value) > 10 && liftVelocity > 0){
            LiftStates.LIFT_MID_RAISING.setStatus(true);
        }
        if(liftPos > LiftConstants.LIFT_MED_JUNCTION.value && liftPos < LiftConstants.LIFT_HIGH_JUNCTION.value && Math.abs(liftPos - LiftConstants.LIFT_HIGH_JUNCTION.value) > 10 && liftVelocity < 0){
            LiftStates.LIFT_HIGH_LOWERING.setStatus(true);
        }
        if(Math.abs(LiftConstants.LIFT_HIGH_JUNCTION.value - liftPos) < 10 && liftVelocity == 0){
            LiftStates.LIFT_HIGH.setStatus(true);
        }
    }
    public void liftToPosition(LiftConstants targetHeight){//TODO: make sure this is async
        //use rfmotor setPosition function to lift in accordance with the enum
        liftMotor.setPosition(targetHeight.value);
        // no conditions
        // log when movement starts & when reach target position
        //logger.log("LiftLog", "Claw lift to " + targetHeight.name() + ", ticks: " + targetHeight.value);
        //async, no use sleep/wait with time, can use multiple processes
        updateLiftStates();
    }
    public void liftToPosition(int targetTickCount){
        liftMotor.setPosition(targetTickCount);
        //logger.log("LiftLog", "Claw lift to " + targetTickCount + " ticks");
        updateLiftStates();
    }
    public void setPower(double power){
        liftTarget=liftMotor.getCurrentPosition();
        if(liftTarget<MAX_LIFT_TICKS&&power>0) {
            liftMotor.setPower(power);
        }
        else if(liftTarget>10&&power<0) {
            liftMotor.setPower(power);
        }
        else{
            liftMotor.setPower(0);
        }
        op.telemetry.addData("LiftPos", liftMotor.getCurrentPosition());
        op.telemetry.addData("LiftVelo", liftMotor.getVelocity());
    }
    public void liftToTarget(){
        liftMotor.setPosition(liftTarget);
    }
    //1 up, -1 down
    public void toggleLiftPosition(int direction){
        if(direction ==1 ){
            //next junction height that is up(use updateLiftState's state to determine)
        }
        else if(direction ==-1){
            //next juntion height that is below
        }
    }

}
