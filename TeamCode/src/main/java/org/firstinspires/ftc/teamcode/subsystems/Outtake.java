package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import java.util.Calendar;
import java.util.Date;

public class Outtake implements Subsystem{
    //Constants
    private double syncFactor=1.05;
    private double armReset_Right = 0.5; private double armReset_Left = 0.5186; // + means up and towards intake
    private double armIntake_Right = 0.51923; private double armIntake_Left = 0.498335;
    //BELOW NOT TESTED YET
    private double armTravel_Right = 0.51263; private double armTravel_Left=0.505265;
    private double armDump_Right = 0.3756; private double armDump_Left = 0.64922;

    private double dumpResetPos = 0.5;
    private double dumpIntakePos= 0.1855;
    private double dumpTravelPos = 0.1785;
    private double dumpCarryPos = 0.6793;
    private double dumpDumpPos = 0.1013;

    //State Machine
    private int swingState;
    /*
     * 0: arm is not moving
     * 1: not yet reached height, wait.
     */
    private int liftState;
    /* //1. wait for intake to move out; 2. go to travel pos, lift the lift 3. at appropriate height, go to dump pos
     * 0: safe to do whatever
     * 1: waiting for intake to lift
     * 2: waiting to reach height with dumper above intake
     */
    private long swingStartTime;
    private long liftStartTime;
    private long tempStartTime;
    private long swingDelay = 500; //miliseconds
    private long liftDelay = 1000;
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
        swingState = 10;
        //lift.goToHt(lift.inchToTicks(0.3));
        dumpServo.setPosition(dumpIntakePos);
        armServo_Right.setPosition(armIntake_Right);
        armServo_Left.setPosition(armIntake_Left);
    }
    public void prepOuttake(){
        liftState = 1;
        liftStartTime = System.currentTimeMillis();
    }
    public void toTravelPos(){
        armServo_Right.setPosition(armTravel_Right);
        armServo_Left.setPosition(armTravel_Left);
        dumpServo.setPosition(dumpTravelPos);
    }
    public void toDumpPos(){
        swingState = 1;
        swingStartTime = System.currentTimeMillis();
        Log.v("StateMach", "start");
        armServo_Right.setPosition(armDump_Right);
        armServo_Left.setPosition(armDump_Left);
    }
    public void dropPixelPos(){
        dumpServo.setPosition(dumpDumpPos);
    }

    //Access functions
    private boolean liftDelayDone(long time){
        return (time - liftStartTime >= liftDelay);
    }
    private boolean swingDelayDone(long time){
        return (time - swingStartTime >= swingDelay);
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
            if(swingDelayDone(time)){
                swingState = 0;
                dumpServo.setPosition(dumpCarryPos);
                Log.v("StateMach", "moving dumper " + (time - swingStartTime));
            }
        }
        if(swingState == 10){
            long time = System.currentTimeMillis();
            if(swingDelayDone(time)){
                swingState = 0;
                lift.goToHt(lift.inchToTicks(0));
                Log.v("StateMach", "moving lift down " + (time - swingStartTime));
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
                Log.v("StateMach", "moving to travelPos " + (time - liftStartTime));
                tempStartTime = System.currentTimeMillis();
                lift.goToLevel(1);  //go to the level where dumper is above intake
            }
        }
        if(liftState == 2){
            long time = System.currentTimeMillis();
            if(lift.armCanSwing()){
                //Log.v("StateMach", "going up"+ (time - tempStartTime));
                liftState=0;
                this.toDumpPos();
            }
        }

    }
}
