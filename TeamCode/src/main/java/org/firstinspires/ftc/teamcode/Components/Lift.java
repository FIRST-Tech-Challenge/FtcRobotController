package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;

import java.util.ArrayList;

@Config
public class Lift {
    private final int MAX_LIFT_TICKS = 1645;
    private final double LIFT_GRAVITY_CONSTANT = 0.07;
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

    //TODO: RFMotor
    private RFMotor liftMotor;
    private Claw LC = new Claw();
    private double liftTarget = 0;
    public static double dfco1 =  0.0122, dfco2 = 1.0/3, dfco3 = 500;
    private ArrayList<Double> coefficients = new ArrayList<>();
    private boolean done = true;
//447,297,173,53,0
    public Lift(){ //constructor
        // hardware map
        Claw LC = new Claw();
        logger.createFile("LiftLog", "Time Component Function Action");
        coefficients.add(dfco1);
        coefficients.add(dfco2);
        coefficients.add(dfco3);
        liftMotor = new RFMotor("liftMotor", DcMotorSimple.Direction.REVERSE, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, true, coefficients, MAX_LIFT_TICKS, 0);
        liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        liftMotor.setTICK_BOUNDARY_PADDING(10);
        liftMotor.setTICK_STOP_PADDING(10);
    }
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

        LiftStates(boolean p_status, String p_name) {
            status = p_status;
            name = p_name;
        }

        public void setStatus(boolean p_status) {
            status = p_status;
            if(p_status==true){
                for(int i = 0; i< ClawExtension.ClawExtensionStates.values().length; i++){
                    if(name != ClawExtension.ClawExtensionStates.values()[i].name()){
                        ClawExtension.ClawExtensionStates.values()[i].setStatus(false);
                    }
                }
            }
        }
    }
    public enum LiftConstants{
        LIFT_HIGH_JUNCTION(1640, false),
        LIFT_MED_JUNCTION(812, false),
        LIFT_LOW_JUNCTION(15, false),
        LIFT_GROUND_JUNCTION(0,false),
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
            if(newVal==true){
                for(int i = 0; i< LiftConstants.values().length; i++){
                    if(this != LiftConstants.values()[i]){
                        LiftConstants.values()[i].setLfc(false);
                    }
                }
            }
        }
        LiftConstants(double num_of_ticks, boolean status) {
            value = num_of_ticks;
            lfcValue = status;
        }
        //make enum for all the tick counts for ground low med high junctions, can set with setGoal(int goal);
    }
    public int getLiftPosition(){
        return liftMotor.getCurrentPosition();
    }
    public double getLiftVelocity(){
        return liftMotor.getVelocity();
    }
    public void updateLiftStates(){
        int liftPos = liftMotor.getCurrentPosition();
        double liftVelocity = liftMotor.getVelocity();
        if(liftPos < 20 && abs(liftVelocity)<20){
            LiftStates.LIFT_GROUND.setStatus(true);
        }
        else{
            LiftStates.LIFT_GROUND.setStatus(false);
        }
//        if(liftPos > 0 && abs(liftPos - LiftConstants.LIFT_GROUND_JUNCTION.value) > liftMotor.getTICK_BOUNDARY_PADDING() && liftVelocity > 0){
//            LiftStates.LIFT_GROUND_RAISING.setStatus(true);
//        }
//        if(liftPos > 0 && abs(liftPos - LiftConstants.LIFT_GROUND_JUNCTION.value) > liftMotor.getTICK_BOUNDARY_PADDING() && liftVelocity < 0){
//            LiftStates.LIFT_GROUND_JUNCTION_LOWERING.setStatus(true);
//        }
//        if(abs(LiftConstants.LIFT_GROUND_JUNCTION.value - liftPos) < liftMotor.getTICK_BOUNDARY_PADDING() && liftVelocity == 0){
//            LiftStates.LIFT_GROUND_JUNCTION.setStatus(true);
//        }
//        if(liftPos > LiftConstants.LIFT_GROUND_JUNCTION.value && liftPos < LiftConstants.LIFT_LOW_JUNCTION.value && abs(liftPos - LiftConstants.LIFT_LOW_JUNCTION.value) > liftMotor.getTICK_BOUNDARY_PADDING() && liftVelocity > 0){
//            LiftStates.LIFT_GROUND_JUNCTION_RAISING.setStatus(true);
//        }
//        if(liftPos > LiftConstants.LIFT_GROUND_JUNCTION.value && liftPos < LiftConstants.LIFT_LOW_JUNCTION.value && abs(liftPos - LiftConstants.LIFT_LOW_JUNCTION.value) > liftMotor.getTICK_BOUNDARY_PADDING() && liftVelocity > 0){
//            LiftStates.LIFT_LOW_LOWERING.setStatus(true);
//        }
//        if(abs(LiftConstants.LIFT_LOW_JUNCTION.value - liftPos) < liftMotor.getTICK_BOUNDARY_PADDING() && liftVelocity == 0){
//            LiftStates.LIFT_LOW.setStatus(true);
//        }
//        if(liftPos > LiftConstants.LIFT_LOW_JUNCTION.value && liftPos < LiftConstants.LIFT_MED_JUNCTION.value && abs(liftPos - LiftConstants.LIFT_MED_JUNCTION.value) > liftMotor.getTICK_BOUNDARY_PADDING() && liftVelocity > 0){
//            LiftStates.LIFT_LOW_RAISING.setStatus(true);
//        }
//        if(liftPos > LiftConstants.LIFT_LOW_JUNCTION.value && liftPos < LiftConstants.LIFT_MED_JUNCTION.value && abs(liftPos - LiftConstants.LIFT_MED_JUNCTION.value) > liftMotor.getTICK_BOUNDARY_PADDING() && liftVelocity < 0){
//            LiftStates.LIFT_MID_LOWERING.setStatus(true);
//        }
//        if(abs(LiftConstants.LIFT_MED_JUNCTION.value - liftPos) < liftMotor.getTICK_BOUNDARY_PADDING() && liftVelocity == 0){
//            LiftStates.LIFT_MID.setStatus(true);
//        }
//        if(liftPos > LiftConstants.LIFT_MED_JUNCTION.value && liftPos < LiftConstants.LIFT_HIGH_JUNCTION.value && abs(liftPos - LiftConstants.LIFT_HIGH_JUNCTION.value) > liftMotor.getTICK_BOUNDARY_PADDING() && liftVelocity > 0){
//            LiftStates.LIFT_MID_RAISING.setStatus(true);
//        }
//        if(liftPos > LiftConstants.LIFT_MED_JUNCTION.value && liftPos < LiftConstants.LIFT_HIGH_JUNCTION.value && abs(liftPos - LiftConstants.LIFT_HIGH_JUNCTION.value) > liftMotor.getTICK_BOUNDARY_PADDING() && liftVelocity < 0){
//            LiftStates.LIFT_HIGH_LOWERING.setStatus(true);
//        }
//        if(abs(LiftConstants.LIFT_HIGH_JUNCTION.value - liftPos) < liftMotor.getTICK_BOUNDARY_PADDING() && liftVelocity == 0){
//            LiftStates.LIFT_HIGH.setStatus(true);
//        }
    }
    public void liftToPosition(LiftConstants targetHeight){//TODO: make sure this is async
        //use rfmotor setPosition function to lift in accordance with the enum
        //if(targetHeight != LiftConstants.LIFT_LOW_JUNCTION && targetHeight != LiftConstants.LIFT_GROUND_JUNCTION && Claw.ClawStates.CLAW_CLOSED.status == true){
            //liftMotor.setPosition(targetHeight.value);
        //}

        double distance = targetHeight.value-liftMotor.getCurrentPosition();

        done = (abs(distance) < liftMotor.getTICK_BOUNDARY_PADDING());
        setLiftTarget(targetHeight.getValue());
        if(!done){
            targetHeight.setLfc(true);
        }
        else{
            targetHeight.setLfc(false);
        }
        // no conditions
        // log when movement starts & when reach target position
        logger.log("/RobotLogs/GeneralRobot", "Lift," + "liftToPosition(LiftConstants)," + "Lifting to " + targetHeight.value + " ticks" + liftMotor.getCurrentPosition());
        //async, no use sleep/wait with time, can use multiple processes
    }
    public boolean isDone(){
        return done;
    }
    public void liftToPosition(int targetTickCount){
        double distance = targetTickCount-liftMotor.getCurrentPosition();

        done = (abs(distance) < liftMotor.getTICK_BOUNDARY_PADDING());
        setLiftTarget(targetTickCount);

        // no conditions
        // log when movement starts & when reach target position
        logger.log("/RobotLogs/GeneralRobot", "Lift," + "liftToPosition(LiftConstants)," + "Lifting to " + targetTickCount + " ticks" + liftMotor.getCurrentPosition());
        //async, no use sleep/wait with time, can use multiple processes
    }
    public void setLiftPower(double power){
        liftTarget=liftMotor.getCurrentPosition();
        if(liftTarget+10<MAX_LIFT_TICKS&&power>0) {
            liftMotor.setPower(power + LIFT_GRAVITY_CONSTANT);
        }
        else if(liftTarget>10&&power<0) {
            liftMotor.setPower(power*0.3 + LIFT_GRAVITY_CONSTANT);
        }
        else{
            liftMotor.setPower(LIFT_GRAVITY_CONSTANT);
        }
        logger.log("LiftLog", "Lift," + "setPower()," + "Lift power set to " + power, true, true, true);
        op.telemetry.addData("ticks", liftMotor.getCurrentPosition());
    }
    public void setLiftRawPower(double rawPower){
        liftMotor.setPower(rawPower);
        liftTarget = liftMotor.getCurrentPosition();
    }
    public void setLiftVelocity(double p_velocity){
        //liftTarget=liftMotor.getCurrentPosition();
        //if(liftTarget<MAX_LIFT_TICKS&&power>0) {
        //    liftMotor.setPower(power);
        //}
        //else if(liftTarget>10&&power<0) {
        liftMotor.setVelocity(p_velocity);
        //}
        //else{
        //    liftMotor.setPower(0);
        //}
        //logger.log("LiftLog", "Claw motor power to " + liftMotor.)
        logger.log("LiftLog", "Lift," + "setPower()," + "Lift power set to " + p_velocity, true, true, true);
    }
    public void resetEncoder(){
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        op.sleep(200);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftTarget=0;
    }
    public void liftToTargetAuto(){
            liftMotor.setPosition(liftTarget);
    }
    public void liftToTarget(){
        if(abs(liftTarget- liftMotor.getCurrentPosition())>100||liftMotor.getVelocity()>20) {
            liftMotor.setPosition(liftTarget);
        }else{
            setLiftPower(0);
        }
    }
    public void setLiftTarget(double p_liftTaret){liftTarget = p_liftTaret;}
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
