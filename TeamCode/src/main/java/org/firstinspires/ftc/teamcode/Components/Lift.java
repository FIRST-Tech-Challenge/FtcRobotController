package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Robots.PwPRobot;

import java.util.ArrayList;


public class Lift {
//    public enum LiftFunctionStates {
//        LIFT_HIGH_JUNCTION(false),
//        LIFT_MED_JUNCTION(false),
//        LIFT_LOW_JUNCTION(false),
//        LIFT_GROUND_JUNCTION(true);
//
//        boolean value;
//        public boolean getValue(){
//            return value;
//        }
//        LiftFunctionStates(boolean status) {
//            value = status;
//        }
//
//    }
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
        LIFT_HIGH_JUNCTION(3500, false),
        LIFT_MED_JUNCTION(2161, false),
        LIFT_LOW_JUNCTION(1035, false),
        LIFT_GROUND_JUNCTION(100,false),
        LIFT_GROUND(0, true);

        double value;
        boolean lfcValue;
        public double getValue(){
            return value;
        }
        public boolean getLfcValue(){
            return lfcValue;
        }
        public void setLfc(boolean newVal){
            this.lfcValue = newVal;
        }
        LiftConstants(double num_of_ticks, boolean status) {
            value = num_of_ticks;
            lfcValue = status;
        }
        //make enum for all the tick counts for ground low med high junctions, can set with setGoal(int goal);
    }
    //TODO: RFMotor
    private RFMotor liftMotor;
    private double MAX_LIFT_TICKS = 3500, liftTarget = 0;
    private double dfco1 = 1.5; private double dfco2 = 150.0;
    private ArrayList<Double> coefficients = new ArrayList<>();
    public Lift(){ //constructor
        // hardware map
        logger.createFile("LiftLog", "Time Junction Ticks");
        coefficients.add(dfco1);
        coefficients.add(dfco2);
        liftMotor = new RFMotor("liftMotor", DcMotorSimple.Direction.FORWARD, DcMotorEx.RunMode.RUN_USING_ENCODER, true, coefficients, MAX_LIFT_TICKS, 0);
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
        //if(targetHeight != LiftConstants.LIFT_LOW_JUNCTION && targetHeight != LiftConstants.LIFT_GROUND_JUNCTION && Claw.ClawStates.CLAW_CLOSED.status == true){
            //liftMotor.setPosition(targetHeight.value);
        //}
        liftMotor.setPosition(targetHeight.value);
        double distance = targetHeight.value-liftMotor.getCurrentPosition();
        boolean done = !(Math.abs(distance) > 20);
        if(!done){
            targetHeight.setLfc(true);
        }
        else{
            targetHeight.setLfc(false);
        }
        op.telemetry.addData("LiftPos", liftMotor.getCurrentPosition());
        op.telemetry.addData("LiftVelo", liftMotor.getVelocity());
        op.telemetry.update();
        // no conditions
        // log when movement starts & when reach target position
        logger.log("LiftLog", "Claw lift to " + targetHeight.value + " ticks");
        //async, no use sleep/wait with time, can use multiple processes
    }
    public void liftToPosition(int targetTickCount){
        liftMotor.setPosition(targetTickCount);
        op.telemetry.addData("LiftPos", liftMotor.getCurrentPosition());
        op.telemetry.addData("LiftVelo", liftMotor.getVelocity());
        op.telemetry.update();
        logger.log("LiftLog", "Claw lift to " + targetTickCount + " ticks");
        updateLiftStates();
    }
    public void setLiftPower(double power){
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
        //logger.log("LiftLog", "Claw motor power to " + liftMotor.)
        op.telemetry.addData("LiftPos", liftMotor.getCurrentPosition());
        op.telemetry.addData("LiftVelo", liftMotor.getVelocity());
        op.telemetry.update();
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
