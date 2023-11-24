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
    private double armResetL = 0.5; private double armResetR = 0.5186; // + means up and towards intake
    private double armIntakeL = 0.50923; private double armIntakeR = 0.508835;
    //BELOW NOT TESTED YET
    private double armTravelL = 0.4381; private double armTravelR=0.583595;
    private double armDumpL = 0.3756; private double armDumpR = 0.64922;
    private long travelDelay = 1000; //miliseconds

    private double dumpResetPos = 0.5;
    private double dumpIntakePos=0.182;
    private double dumpTravelPos = 0.4885;
    private double dumpCarryPos = 0.6793;
    private double dumpDumpPos = 0;

    //State Machine
    private int swingState;
    /*
     * 0: arm is not moving
     * 1: not yet reached height, wait.
     * 2: height reached, rotate dumper.
     */
    private long moveStartTime;

    //Hardware
    private Servo armServoL;
    private Servo armServoR;  //    '/[L^R]\'
    private Servo dumpServo;
    private DualMotorLift lift;
    private Telemetry telemetry;


    public Outtake(Robot robot) {
        this.telemetry = telemetry;
        //lift = new DualMotorLift(robot, telemetry, DualMotorLift.Mode.RIGHT_FOLLOW_LEFT);
        armServoL = robot.getServo("armServoL");
        armServoR = robot.getServo("armServoR");
        dumpServo = robot.getServo("dumpServo");
    }

    //TODO: convert left/right positions?
    public double leftPosToRightPos(double leftPos){
        return 0; //NOT DONE
    }


    //Reset Functions
    public void resetArmPos(){
        armServoL.setPosition(armResetL);
        armServoR.setPosition(armResetR);
    }

    public void resetDumpPos(){
        dumpServo.setPosition(dumpResetPos);
    }

    //Action functions
    public void moveR(double d){
        //TODO: for Servo Sync only
        armServoR.setPosition(armServoR.getPosition()+(0.001*d));
    }
    public void moveArm(double d){
        armServoL.setPosition(armServoL.getPosition()+(0.001*-d)); //2 degrees??
        armServoR.setPosition(armServoR.getPosition()+(0.001*d*syncFactor));
    }
    public void moveDumper(double d){
        dumpServo.setPosition(dumpServo.getPosition()+(0.005*d)); //2 degrees??
    }
    public void moveSlide(double inches){ //INCHES
        lift.goToHt(lift.inchToTicks(inches));
    }
    public void toIntakePos(){
        swingState = 0;
        //lift.goToLevel(0);
        dumpServo.setPosition(dumpIntakePos);
        armServoL.setPosition(armIntakeL);
        armServoR.setPosition(armIntakeR);
    }
    public void toTravelPos(){
        armServoL.setPosition(armTravelL);
        armServoR.setPosition(armTravelR);
        dumpServo.setPosition(dumpTravelPos);
    }
    public void toDumpPos(int level){
        swingState = 1;
        moveStartTime = System.currentTimeMillis();
        Log.v("StateMach", "start");
        //lift.goToLevel(level);
        armServoL.setPosition(armDumpL);
        armServoR.setPosition(armDumpR);
    }
    public void dropPixelPos(){
        dumpServo.setPosition(dumpDumpPos);
    }

    //Access functions
    private boolean delayDone(long time){
        return (time - moveStartTime >= travelDelay);
    }
    public double getRightServoPos() {
        return armServoR.getPosition();
    }
    public double getLeftServoPos(){
        return armServoL.getPosition();
    }

    public double getDumperPos(){
        return dumpServo.getPosition();
    }

    @Override
    public void update(TelemetryPacket packet) {
        //lift.update(packet);
        //state machine
        /*
         * 0: arm is not moving
         * 1: not yet reached height, wait.
         */
        if(swingState == 1){
            long time = System.currentTimeMillis();
            if(delayDone(time)){
                swingState =0;
                dumpServo.setPosition(dumpCarryPos);
                Log.v("StateMach", "moving dumper " + (time - moveStartTime));
            }
        }

    }
}
