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
    public static double HT_TO_SWING_AUTO = 8.0;
     //0.09;
    //Constants
    private double syncFactor=1.05;
    private double armReset_Right = 0.5; private double armReset_Left = 0.5186; // + means up and towards intake 0.5, 0.5186
    public static double armIntake_Right = 0.49923; public static double armIntake_Left = 0.518335;
    //private double armTravel_Right = 0.51263; private double armTravel_Left= 0.505265;//0.51263, 0.505265
    public static double armTravel_Right = 0.512; public static double armTravel_Left= 0.505;//0.51263, 0.505265
    private double armDump_Right = 0.44; private double armDump_Left = 0.57;

    public static double armIntake_RightA = 0.5; public static double armIntake_LeftA = 0.52;
    private double armTravel_RightA = 0.4950; private double armTravel_LeftA= 0.52;//0.51263, 0.505265
    public static double armTravelDown_RightA = 0.505; public static double armTravelDown_LeftA= 0.515;//0.51263, 0.505265

    private double armDumpLow_Right = 0.462; private double armDumpLow_Left = 0.5536;
    private double armHang_Right = 0.492; private double armHang_Left = 0.53; //0.525,0.492

    private double dumpResetPos = 0.47;//0.5
    private double dumpIntakePos= 0.59;
    private double dumpTravelPos = 0.40; //0.37; //0.1785;
    private double dumpCarryPos = 0.4; //0.6793;
    private double dumpLiftCarryPos = 0.5;
    private double dumpDumpPos = 0.31; //0.29

    private double dumpResetPosA = 0.58; //0.45;
    private double dumpIntakePosA= 0.58; //0.45;
    private double dumpTravelPosA = 0.50; //0.37;
    private double dumpCarryPosA = 0.45; //0.32;
    private double dumpDumpPos_auto = 0.31;

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
    private long tempDelay = 1000;

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

    public void setDumpServoPos(double pos) {
        dumpServo.setPosition(pos);
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
        Log.v("StateMach", "toIntakePos() called");
        swingState = 10;
        swingStartTime = System.currentTimeMillis();
        //armToTravelPos();
        //dumperToIntakePos();
    }

    public void toIntakePosAuto(){
        Log.v("StateMach", "toIntakePos() called");
        swingState = 20;
        swingStartTime = System.currentTimeMillis();
        //armToTravelPos();
        //dumperToIntakePos();
    }
    public void prepOuttake(){
        liftState = 1;
        liftStartTime = System.currentTimeMillis();
        lift.goToLevel(1);
        Log.v("StateMach", "prepOuttake() called");
    }
    public void prepOuttakeAuto(){
        liftState = 21;
        liftStartTime = System.currentTimeMillis();
        Log.v("StateMach", "prepOuttakeAuto() called");
    }
    public void armToTravelPos(){
        armServo_Right.setPosition(armTravel_Right);
        armServo_Left.setPosition(armTravel_Left);
        Log.v("StateMach", "armToTravelPos() called");
    }
    public void armToTravelPosAuto(){
        armServo_Right.setPosition(armTravelDown_RightA);
        armServo_Left.setPosition(armTravelDown_LeftA);
        Log.v("StateMach", "armToTravelPosAuto() called");
    }
    public void armToBackdropPos(){
        armServo_Right.setPosition(armTravel_Right-0.08);
        armServo_Left.setPosition(armTravel_Left+0.08);
        Log.v("StateMach", "armToBackdropPos() called");
    }

    public void armToBackdropPosA(){
        armServo_Right.setPosition(armTravel_Right-0.07);
        armServo_Left.setPosition(armTravel_Left+0.07);
        Log.v("StateMach", "armToBackdropPosA() called");
    }

    public void dumperToTravelPos() {
        dumpServo.setPosition(dumpTravelPos);
        Log.v("StateMach", "dumperToTravelPos() called");
    }
    public void dumperToTravelPosAuto() {
        dumpServo.setPosition(dumpTravelPosA);
        Log.v("StateMach", "dumperToTravelPos() called");
    }
    public void dumperToIntakePos() {
        dumpServo.setPosition(dumpIntakePos);
        Log.v("StateMach", "dumperToIntake() called");
    }
    public void dumperToIntakePosAuto() {
        dumpServo.setPosition(dumpIntakePosA);
        Log.v("StateMach", "dumperToIntakeAuto() called");
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
        Log.v("StateMach", "toDunpPos() called: swing start");
        armServo_Right.setPosition(armDump_Right);
        armServo_Left.setPosition(armDump_Left);
    }

    public void low_toDumpPos(){
        swingState = 1;
        swingStartTime = System.currentTimeMillis();
        Log.v("StateMach", "toDunpPos() called: swing start");
        armToBackdropPosA();
        //armServo_Right.setPosition(armDumpLow_Right);
        //armServo_Left.setPosition(armDumpLow_Left);
    }

    public void dropPixelPos(){
        dumpServo.setPosition(dumpDumpPos);
        Log.v("StateMach", "dump servo to dumpDumpPos");
    }

    public void dropPixelPosAuto(){
        dumpServo.setPosition(dumpDumpPos_auto);
        Log.v("StateMach", "dump servo to dumpDumpPos_auto");
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
                Log.v("StateMach", "swing done. moving dumper to CarryPos " + (time - swingStartTime));
            }
        }
        if(swingState == 10){ // = lower to intake pos
            long time = System.currentTimeMillis();
            if(armCanSwing(time)){
                swingState = 11;
                this.armToTravelPos();
                this.dumperToIntakePos();
                Log.v("StateMach", "lift moving to travelPos. swingState 10 " + (time - liftStartTime));
                telemetry.addLine("swingState == 10, arm in travel position, dumper in intake position");
                tempStartTime = System.currentTimeMillis();
                lift.goToHt(lift.inchToTicks(lift.LEVEL_HT[0]));
            }
        }
        if(swingState == 11){
            long time = System.currentTimeMillis();
            if(swingDelayDone(time, swingDownDelay)){
                swingState = 12;
                lift.goToHt(lift.inchToTicks(-0.3));
                Log.v("StateMach", "lower swing tate: 11 -> 12. lift.goToHt " + 0);
                telemetry.addLine("swingState == 11");
            }
        }
        if(swingState == 12){
            if(lift.getPosition()<2){
                swingState = 0;
                armServo_Right.setPosition(armIntake_Right);
                armServo_Left.setPosition(armIntake_Left);
                dumpServo.setPosition(dumpIntakePos);
                lift.goToHt(lift.inchToTicks(-0.3));
                Log.v("StateMach", "lower swing tate: 10 -> 22. lift.goToHt " + HT_TO_SWING_AUTO);
                telemetry.addLine("swingState == 12. arm servos to arm intake position, dump servo to intake position");
            }
        }

        // For Auto
        if(swingState == 20){ // = lower to intake pos
            long time = System.currentTimeMillis();
            if(armCanSwing(time)){
                swingState = 21;
                this.armToTravelPosAuto();
                this.dumperToIntakePosAuto();
                Log.v("StateMach", "lift moving to travelPos. swingState 20 " + (time - liftStartTime));
                tempStartTime = System.currentTimeMillis();
                lift.goToHt(lift.inchToTicks(lift.LEVEL_HT[0]));
            }
        }
        if(swingState == 21){
            long time = System.currentTimeMillis();
            if(swingDelayDone(time, swingDownDelay)){
                swingState = 22;
                lift.goToHt(lift.inchToTicks(-0.3));
                Log.v("StateMach", "lower swing tate: 21 -> 22. lift.goToHt " + 0);
            }
        }
        if(swingState == 22){
            if(lift.getPosition()<0.8){
                swingState = 0;
                armServo_Right.setPosition(armIntake_RightA);
                armServo_Left.setPosition(armIntake_LeftA);
                dumpServo.setPosition(dumpIntakePosA);
                lift.goToHt(lift.inchToTicks(-0.3));
                Log.v("StateMach", "lower swing tate: 20 -> 22. lift.goToHt " + 0);
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
                dumpServo.setPosition(dumpLiftCarryPos);
                //this.armToBackdropPos();
                //this.dumperToTravelPos();
                Log.v("StateMach", "dumper move to carry pos, liftState 1 " + (time - liftStartTime));
                tempStartTime = System.currentTimeMillis();
                //lift.goToLevel(1);  //go to the level where dumper is above intake
            }
        }
        if(liftState == 2){
            long time = System.currentTimeMillis();
            if(time - tempStartTime >= 500){
                Log.v("StateMach", "liftState = 2. Start toCarryPos");
                this.armToBackdropPos();
                this.dumperToTravelPos();
                liftState=3;
            }
        }
        if(liftState == 3){
            long time = System.currentTimeMillis();
            if(lift.armCanSwing() && time - tempStartTime >= 500){
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
                this.armToTravelPos();
                this.dumperToTravelPosAuto();
                Log.v("StateMach", "lift moving to travelPos, liftState 21 " + (time - liftStartTime));
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
                this.low_toDumpPos();
                this.setDumpServoPos(dumpCarryPosA);
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

        //Log.v("StateMach", "lift HT " + lift.getPosition());
    }
}
