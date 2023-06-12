package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static java.lang.Double.min;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;

import java.util.ArrayList;

@Config
public class Lift {
    public static int MAX_LIFT_TICKS = 1370;
    private final double LIFT_GRAVITY_CONSTANT = 0.075;
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
    private double liftTarget = 0,dfco1 = 40, dfco2 = 3, dfco3 = 0;
    private ArrayList<Double> coefficients = new ArrayList<>();
    private boolean done = true;
    private double lastManualTime = 0.0;
    double[] coneStack = {500*0.6,410*0.6-0,315*0.6-10,180*0.6 - 20};
    ;
    private int stackLevel = 0;
    private double lastStackTime =0;

    public Lift() { //constructor
        // hardware map
        coefficients.add(dfco1);
        if(isTeleop){
            dfco2=0.5;
        }
        else{
            dfco2=2;
        }
        coefficients.add(dfco2);
        coefficients.add(dfco3);
        liftMotor = new RFMotor("liftMotor", DcMotorSimple.Direction.REVERSE, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER,
                false, coefficients, MAX_LIFT_TICKS, -30);
        if (!isTeleop) {
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(isTeleop){

        }
        liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
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
            if (p_status == true) {
                for (int i = 0; i < Lift.LiftStates.values().length; i++) {
                    if (name != Lift.LiftStates.values()[i].name()) {
                        Lift.LiftStates.values()[i].setStatus(false);
                    }
                }
            }
        }
    }

    public enum LiftConstants {
        LIFT_HIGH_JUNCTION(MAX_LIFT_TICKS - 80, false),
        LIFT_MED_JUNCTION(860*.8-100, false),
        LIFT_LOW_JUNCTION(15*.75, false),
        LIFT_GROUND_JUNCTION(0, false),
        LIFT_GROUND(-10, true);

        double value;
        boolean lfcValue;

        public double getValue() {
            return value;
        }

        public boolean getLfcValue() {
            return lfcValue;
        }

        public void setLfc(boolean newVal) {
            this.lfcValue = newVal;
            if (newVal == true) {
                for (int i = 0; i < LiftConstants.values().length; i++) {
                    if (this != LiftConstants.values()[i]) {
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

    public int getLiftPosition() {
        return liftMotor.getCurrentPosition();
    }

    public double getLiftVelocity() {
        return liftMotor.getVelocity();
    }

    public void setPeak(){
        liftMotor.setCurrentPosition(1260);
    }

    public double getStackLevelHeight(int i){
        return coneStack[i];
    }

    public void updateLiftStates() {
        int liftPos = liftMotor.getCurrentPosition();
        done = (abs(liftTarget-liftPos) < liftMotor.getTICK_BOUNDARY_PADDING());
        op.telemetry.addData("ticks", liftPos);
        double liftVelocity = liftMotor.getVelocity();
        if (liftPos < 20 && abs(liftVelocity) < 20) {
            LiftStates.LIFT_GROUND.setStatus(true);
        } else {
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

    public void liftToPosition(LiftConstants targetHeight) {//TODO: make sure this is async
        //use rfmotor setPosition function to lift in accordance with the enum
        //if(targetHeight != LiftConstants.LIFT_LOW_JUNCTION && targetHeight != LiftConstants.LIFT_GROUND_JUNCTION && Claw.ClawStates.CLAW_CLOSED.status == true){
        //liftMotor.setPosition(targetHeight.value);
        //}
        if(targetHeight.value!=liftMotor.getTarget()) {
            logger.log("/RobotLogs/GeneralRobot", "Lift," + "liftToPosition(LiftConstants)," + "Lifting to " + targetHeight.value /*+ " ticks" + liftMotor.getCurrentPosition()*/);
        }
        setLiftTarget(targetHeight.getValue());
        // no conditions
        // log when movement starts & when reach target position
        //async, no use sleep/wait with time, can use multiple processes
    }

    public boolean isDone() {
        return done;
    }

    public void liftToPosition(int targetTickCount) {
        if(targetTickCount!=liftMotor.getTarget()) {
            logger.log("/RobotLogs/GeneralRobot", "Lift," + "liftToPosition(LiftConstants)," + "Lifting to " + targetTickCount /*+ " ticks" + liftMotor.getCurrentPosition()*/);
        }
        setLiftTarget(min(targetTickCount,MAX_LIFT_TICKS));

        // no conditions
        // log when movement starts & when reach target position
//        logger.log("/RobotLogs/GeneralRobot", "Lift," + "liftToPosition(LiftConstants)," + "Lifting to " + liftTarget /*+ " ticks" + liftMotor.getCurrentPosition()*/);
        //async, no use sleep/wait with time, can use multiple processes
    }

    private boolean liftZeroLogged = false;

    public void setLiftPower(double power) {
        liftTarget = liftMotor.getCurrentPosition();
        if ((liftTarget + 10 < MAX_LIFT_TICKS && power > 0) || (liftTarget > 10 && power < 0)) {
            liftMotor.setPower(power + LIFT_GRAVITY_CONSTANT);
            logger.log("/RobotLogs/GeneralRobot", "Lift," + "setPower()," + "Lift power set to " + power, true);
            liftZeroLogged = false;
        } else {
            liftMotor.setPower(LIFT_GRAVITY_CONSTANT);
            if (liftZeroLogged == false) {
                logger.log("/RobotLogs/GeneralRobot", "Lift," + "setPower()," + "Lift power set to 0", true);
                liftZeroLogged = true;
            }
        }
        op.telemetry.addData("ticks", liftMotor.getCurrentPosition());
    }

    public void updateLastManualTime() {
        lastManualTime = time;
    }

    public void setLiftRawPower(double rawPower) {
        liftMotor.setPower(rawPower);
        liftTarget = liftMotor.getCurrentPosition();
    }

    public void setLiftVelocity(double p_velocity) {
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

        //logger.log("GeneralRobotLog", "Claw motor power to " + liftMotor.)
        logger.log("/RobotLogs/GeneralRobot", "Lift," + "setPower()," + "Lift power set to " + p_velocity, true, true, true);
    }

    public void resetEncoder() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        op.sleep(200);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftTarget = 0;
        liftMotor.setCurrentPosition(0);
    }
    public double getLiftTarget(){
        return liftTarget;
    }

    public void liftToTargetAuto() {
        liftMotor.setPosition(liftTarget);
    }

    private boolean LTTZeroLogged = false;

    public void liftToTarget() {
        if (time- lastManualTime > 0.2) {
            liftMotor.setPosition(liftTarget);
            //logger.log("/RobotLogs/GeneralRobot", "Lift," + "liftToTarget()," + "Lifting to Target of:" + liftTarget + " ticks" + logger.loopcounter, true);
            //logger.log("/RobotLogs/GeneralRobot", "Lift," + "liftToTarget()," + "Target: " + liftTarget + " ticks | Current Position: " + liftMotor.getCurrentPosition() + " | Velocity: " + liftMotor.getVelocity(), true);
            LTTZeroLogged = false;
        } else if (time- lastManualTime < .2 && liftMotor.getPower() == liftMotor.getGRAVITY_CONSTANT()) {
            setLiftTarget(liftMotor.getCurrentPosition());
        } else {
            //if (LTTZeroLogged == false) {
                //logger.log("/RobotLogs/GeneralRobot", "liftToTarget()," + "Lifting to Target of:" + liftTarget + " ticks", true);
                //LTTZeroLogged = true;
            //}
            setLiftPower(0);
        }

    }
    public void setStacklevel(int i){
        setLiftTarget(coneStack[i]);
    }

    public void iterateConeStackDown() {
        if(time-lastStackTime>0.3) {
            if (stackLevel != 3) {
                stackLevel++;
            }
//            op.telemetry.addData("stackLevel", stackLevel);
            setLiftTarget(coneStack[stackLevel]);
            lastStackTime=time;
        }
    }

    public void iterateConeStackUp() {
        if(time-lastStackTime>0.2) {
            if (stackLevel != 0) {
                stackLevel--;
            }
//            op.telemetry.addData("stackLevel", stackLevel);
            setLiftTarget(coneStack[stackLevel]);
//            setLiftTarget(coneStack[stackLevel]);
            lastStackTime=time;
        }
    }
    public int getStackLevel(){
        return stackLevel;
    }
    public double getStackPos(){
        return coneStack[stackLevel];
    }

    public void raiseLiftOffStack(){
        setLiftTarget(coneStack[stackLevel]+500);
    }

    public void setLiftTarget(double p_liftTarget) {
        liftTarget = min(MAX_LIFT_TICKS,p_liftTarget);
//        logger.log("/RobotLogs/GeneralRobot", "Lift," + "setLiftTarget()," + "Lift target set to:" + p_liftTarget + " ticks", true);
    }

    //1 up, -1 down
    public void toggleLiftPosition(int direction) {
        if (direction == 1) {
            //next junction height that is up(use updateLiftState's state to determine)
        } else if (direction == -1) {
            //next juntion height that is below
        }
    }

}
