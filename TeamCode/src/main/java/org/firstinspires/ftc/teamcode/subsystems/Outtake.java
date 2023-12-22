package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import java.util.Calendar;
import java.util.Date;

@Config
public class Outtake implements Subsystem{
    public static double HT_TO_SWING_AUTO = 3.0;
    public static double dumpDumpPos_auto = 0.09;
    //Constants
    private double syncFactor=1.05;
    private double armReset_Right = 0.5; private double armReset_Left = 0.5186; // + means up and towards intake
    private double armIntake_Right = 0.51923; private double armIntake_Left = 0.498335;
    //BELOW NOT TESTED YET
    private double armTravel_Right = 0.51263; private double armTravel_Left=0.505265;
    private double armDump_Right = 0.3756; private double armDump_Left = 0.64922;
    private double armHang_Right = 0.525; private double armHang_Left = 0.492;

    private double dumpResetPos = 0.5;
    private double dumpIntakePos= 0.1855;
    private double dumpTravelPos = 0.1785;
    private double dumpCarryPos = 0.6793;
    private double dumpDumpPos = 0.1013;

    //State Machine
    public int swingState;
    /*
     * 0: arm is not moving
     * 1: not yet reached height, wait.
     */
    public int liftState;
    /* //1. wait for intake to move out; 2. go to travel pos, lift the lift 3. at appropriate height, go to dump pos
     * 0: safe to do whatever
     * 1: waiting for intake to lift
     * 2: waiting to reach height with dumper above intake
     * 10: wating for reach 0 (and then put dumper to input position)
     */

    public int hangState=0;
    private long swingStartTime;
    private long liftStartTime;
    private long tempStartTime;
    private long swingDelay = 500; //miliseconds
    private long swingDownDelay = 1200; //miliseconds
    private long liftDelay = 400; // time it takes for intake to move away
    private long tempDelay = 5000;

    //Hardware
    private Servo armServo_Right;
    private Servo armServo_Left;  //    '/[L^R]\'
    private Servo dumpServo;
    public DualMotorLift lift;
    private Telemetry telemetry;


    public Outtake(Robot robot, Telemetry telemetry) {
        this.telemetry = telemetry;
        lift = new DualMotorLift(robot, telemetry, DualMotorLift.Mode.BOTH_MOTORS_PID);
        armServo_Right = robot.getServo("armServo_Right");
        armServo_Left = robot.getServo("armServo_Left");
        dumpServo = robot.getServo("dumpServo");
        toIntakePos();
    }

    //TODO: convert left/right positions?
    public double leftPosToRightPos(double leftPos){
        return 0; //NOT DONE
    }


    //Reset Functions
    public void resetArmPos(){
        armServo_Right.setPosition(armReset_Right);
        armServo_Left.setPosition(armReset_Left);
    }

    public void resetDumpPos(){
        dumpServo.setPosition(dumpResetPos);
    }

    //Action functions
    public void moveL(double d){
        //TODO: for Servo Sync only
        armServo_Left.setPosition(armServo_Left.getPosition()+(0.001*d));
    }
    public void moveArm(double d){
        armServo_Right.setPosition(armServo_Right.getPosition()+(0.001*-d)); //2 degrees??
        armServo_Left.setPosition(armServo_Left.getPosition()+(0.001*d*syncFactor));
    }
    public void moveDumper(double d){
        dumpServo.setPosition(dumpServo.getPosition()+(0.005*d)); //2 degrees??
    }
    public void moveSlide(double inches){ //outputs ticks to REL offset
        lift.goToRelativeOffset(inches);
    }
    public void toIntakePos(){
        //Log.v("StateMach", "toIntakePos() called");
        swingState = 10;
        swingStartTime = System.currentTimeMillis();
        toTravelPos();
    }
    public void prepOuttake(){
        liftState = 1;
        liftStartTime = System.currentTimeMillis();
        //Log.v("StateMach", "prepOuttake() called");
    }
    public void prepOuttakeAuto(){
        liftState = 21;
        liftStartTime = System.currentTimeMillis();
        //Log.v("StateMach", "prepOuttakeAuto() called");
    }
    public void toTravelPos(){
        armServo_Right.setPosition(armTravel_Right);
        armServo_Left.setPosition(armTravel_Left);
        dumpServo.setPosition(dumpTravelPos);
        //Log.v("StateMach", "toTravelPos() called");
    }
    public void toHangPos(){
        armServo_Right.setPosition(armHang_Right);
        armServo_Left.setPosition(armHang_Left);
        dumpServo.setPosition(dumpResetPos);
    }
    public void prepHang(){
        hangState = 1;
    }
    public void toDumpPos(){
        swingState = 1;
        swingStartTime = System.currentTimeMillis();
        //Log.v("StateMach", "toDunpPos() called: swing start");
        armServo_Right.setPosition(armDump_Right);
        armServo_Left.setPosition(armDump_Left);
    }

    public void dropPixelPos(){
        dumpServo.setPosition(dumpDumpPos);
        //Log.v("StateMach", "dump servo to dumpDumpPos");
    }

    public void dropPixelPosAuto(){
        dumpServo.setPosition(dumpDumpPos_auto);
        //log.v("StateMach", "dump servo to dumpDumpPos_auto");
    }

    //Access functions
    private boolean liftDelayDone(long time){
        return (time - liftStartTime >= liftDelay);
    }
    private boolean swingDelayDone(long time, long delay){
        return (time - swingStartTime >= delay);
    }
    private boolean armCanSwing(long time){
        return (time - tempStartTime >= tempDelay);
    }

    public double get_LeftServoPos() {
        return armServo_Left.getPosition();
    }
    public double get_RightServoPos(){
        return armServo_Right.getPosition();
    }

    public double getDumperPos(){
        return dumpServo.getPosition();
    }
    public double getLiftPos(){ return lift.getPosition(); }
    public double getLiftPower() {return lift.getPIDPower(); }

    @Override
    public void update(TelemetryPacket packet) {
        lift.update(packet);
        //lift.update(packet);
        //state machine
        /*
         * 0: arm is not moving
         * 1: not yet reached height, wait.
         * 5:
         */
        if(swingState == 1){
            long time = System.currentTimeMillis();
            if(swingDelayDone(time, swingDelay)){
                swingState = 0;
                dumpServo.setPosition(dumpCarryPos);
                //Log.v("StateMach", "swing done. moving dumper to CarryPos " + (time - swingStartTime));
            }
        }
        if(swingState == 10){ // = lower to intake pos
            long time = System.currentTimeMillis();
            if(armCanSwing(time)){
                swingState = 11;
                this.toTravelPos();
                //Log.v("StateMach", "lift moving to travelPos " + (time - liftStartTime));
                tempStartTime = System.currentTimeMillis();
                lift.goToHt(lift.inchToTicks(lift.LEVEL_HT[0]));
            }
        }
        if(swingState == 11){
            long time = System.currentTimeMillis();
            if(swingDelayDone(time, swingDelay)){
                swingState = 12;
                lift.goToHt(lift.inchToTicks(-0.3));
                //Log.v("StateMach", "lower swing tate: 11 -> 12. lift.goToHt " + 0);
            }
        }
        if(swingState == 12){
            if(lift.getPosition()<2){
                swingState = 0;
                armServo_Right.setPosition(armIntake_Right);
                armServo_Left.setPosition(armIntake_Left);
                dumpServo.setPosition(dumpIntakePos);
                lift.goToHt(lift.inchToTicks(-0.3));
                //Log.v("StateMach", "lower swing tate: 10 -> 22. lift.goToHt " + HT_TO_SWING_AUTO);
            }
        }


        /*
         * 0: safe to do whatever
         * 1: waiting for intake to move: when done, go to travelPos and raise slide
         * 2: waiting to reach height with dumper above intake: when done, swing arm to dump pos
         */
        if(liftState == 1){
            long time = System.currentTimeMillis();
            if(liftDelayDone(time)){
                liftState = 2;
                this.toTravelPos();
                Log.v("StateMach", "lift moving to travelPos " + (time - liftStartTime));
                tempStartTime = System.currentTimeMillis();
                lift.goToLevel(1);  //go to the level where dumper is above intake
            }
        }
        if(liftState == 2){
            long time = System.currentTimeMillis();
            if(lift.armCanSwing()){
                Log.v("StateMach", "liftState = 0. Start toDumpPos");
                liftState=0;
                this.toDumpPos();
            }
        }

        // For auto
        if(liftState == 21){
            long time = System.currentTimeMillis();
            if(liftDelayDone(time)){
                liftState = 22;
                this.toTravelPos();
                Log.v("StateMach", "lift moving to travelPos " + (time - liftStartTime));
                tempStartTime = System.currentTimeMillis();
                //lift.goToLevel(1);  //go to the level where dumper is above intake
                lift.goToHt(lift.inchToTicks(HT_TO_SWING_AUTO));
                Log.v("StateMach", "liftState: 21 -> 22. lift.goToHt " + HT_TO_SWING_AUTO);
            }
        }
        if(liftState == 22){
            long time = System.currentTimeMillis();
            if(lift.armCanSwingAuto()){
                Log.v("StateMach", "liftState = 0. Start toDumpPos");
                liftState=0;
                this.toDumpPos();
            }
        }
/*
        if(liftState == 10){
            if(lift.getPosition() < 1.0) {
                Log.v("StateMach", "lift down reached. reset arm pos");
                armServo_Right.setPosition(armIntake_Right);
                armServo_Left.setPosition(armIntake_Left);
                dumpServo.setPosition(dumpIntakePos);
                liftState = 0;
            }
        }*/
        if(hangState == 1){
            long time = System.currentTimeMillis();
            if(liftDelayDone(time)){
                hangState = 0;
                this.toHangPos();
                Log.v("StateMach", "moving to travelPos " + (time - liftStartTime));
                tempStartTime = System.currentTimeMillis();
                lift.goToLevel(0);  //go to the level where dumper is above intake
            }
        }
    }
}
