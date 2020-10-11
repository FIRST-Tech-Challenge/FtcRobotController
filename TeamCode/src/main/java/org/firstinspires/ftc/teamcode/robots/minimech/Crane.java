package org.firstinspires.ftc.teamcode.robots.minimech;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.abs;

/**
 * Created by 2938061 on 11/10/2017.
 */

public class Crane {

    //number of ticks per revolution REV HD motor: 2240
    //number of ticks per revolution REV Core    : 288

    DcMotor elbow = null;
    DcMotor extendABob = null;

    Servo intakeRight = null;
    Servo intakeLeft = null;
    Servo hook = null;
    Servo intakeGate = null;

    int elbowPosInternal = 0;
    int elbowPos = 0;
    double elbowPwr = 1;

    int extendABobPosInternal = 0;
    int extendABobPos = 0;
    double extendABobPwr = 1;

    int intakeState = 3;
    boolean beltToElbowEnabled;

    public int servoHooked;
    public int servoUnhooked;

    int servoGateOpen;
    int servoGateClosed;

    public double intakePwr;
    //normal Teleop encoder values
    public int pos_preIntake;
    public int pos_Intake ;
    public int pos_Deposit;
    public int pos_reverseIntake;
    public int pos_reversePreDeposit;
    public int pos_reverseDeposit;
    public int pos_reverseSafeDrive;
    public int pos_PartialDeposit;
    public int pos_SafeDrive;

    //autonomous encoder values
    public int pos_AutoPark;
    public int pos_autonPrelatch;

    //end game encoder values
    public int pos_prelatch;
    public int pos_latched;
    public int pos_postlatch;
    public int glide = 80;
    public int autodepotthingy=350;

    //.374

    //belt extension encoder values
    public  int extendDeposit;
    public  int extendMax;
    public  int extendMid;
    public  int extendLow; //clears hook and good for retracting prior to deposit without tipping robot
    public  int extendMin;  //prevent crunching collector tray
    public  int extendPreLatch = extendMax;

    public int stow = 650;


    private boolean hookUp = true;
    private boolean gateOpen  = true;

    //filler value; needs to be updated to reflect actual ratio
    public double ticksPerDegree = 22.3296703;

    public boolean active = true;

    public Crane(DcMotor elbow, DcMotor extendABob, Servo hook, Servo intakeGate){

        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setTargetPosition(elbow.getCurrentPosition());
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setDirection(DcMotorSimple.Direction.REVERSE);

        extendABob.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendABob.setTargetPosition(extendABob.getCurrentPosition());
        extendABob.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendABob.setDirection(DcMotorSimple.Direction.REVERSE);
        //extendABobRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //intakeGate.setDirection(Servo.Direction.REVERSE);

        this.elbow = elbow;
        this.extendABob = extendABob;
        this.hook = hook;
        this.intakeGate = intakeGate;


        intakePwr = .3; //.35;
        //normal Teleop encoder values
        pos_preIntake = 3600;
        pos_Intake   = 3900;
        pos_Deposit  = 1520;
        pos_reverseIntake = 1407;
        pos_reversePreDeposit=1408;
        pos_reverseDeposit = 3400;
        pos_reverseSafeDrive = 1000;
        pos_PartialDeposit = 1700;
        pos_SafeDrive = 800;

        //autonomous encoder values
        pos_AutoPark = pos_SafeDrive + 500;
        pos_autonPrelatch = 2950;

        //end game encoder values
        pos_prelatch = 2000;
        pos_latched = 2764;
        pos_postlatch = 1240;

        servoGateOpen = 1700;
        servoGateClosed = 900;

        servoHooked = 1800;
        servoUnhooked = 1300;

        //belt extension encoder values
        extendDeposit = 1489;
        extendMax = 2960;
        extendMid= 980;
        extendLow = 650; //clears hook and good for retracting prior to deposit without tipping robot
        extendMin = 300;  //prevent crunching collector tray
    }


    public void update(){
        if(active && elbowPosInternal!=elbowPos) { //don't keep updating if we are retractBelt to target position
            elbowPosInternal = elbowPos;
            elbow.setTargetPosition(elbowPos);
            elbow.setPower(elbowPwr);
        }
        if(active && extendABobPosInternal!=extendABobPos) { //don't keep updating if we are retractBelt to target position
            extendABobPosInternal = extendABobPos;
            extendABob.setTargetPosition(extendABobPos);
            extendABob.setPower(extendABobPwr);
        }
        updateIntake();
        updateBeltToElbow();
    }

    public void updateBeltToElbow() {
        if(beltToElbowEnabled) {
            setElbowTargetPos(beltToElbow(getExtendABobCurrentPos(), 0)-40);
        }
    }

    public void setBeltToElbowModeEnabled() {
        beltToElbowEnabled = true;
    }

    public void setBeltToElbowModeDisabled() {
        beltToElbowEnabled = false;
    }

    public int elbowToBelt(int elbow, int offset){
        return (int)(4.5*(elbow+ offset)) +620;
    }

    public int beltToElbow(int belt, int offset){
        return (int)(2.0/9 * ((belt+offset)-620)) ;
    }

    public void updateIntake() {
        switch(intakeState) {
            case 0:
                stopIntake();
                break;
            case 1:
                collect();
                break;
            case 2:
                eject();
                break;
            case 3:
            default:
                //do nothing
                break;
        }
    }

    public void setIntakeModeOff() {
        intakeState = 0;
    }
    public void setIntakeModeIn() {
        intakeState = 1;
    }
    public void setIntakeModeOut() {
        intakeState = 2;
    }
    public void setIntakeModeManual() {
        intakeState = 3;
    }


    public void hookOn(){

        hook.setPosition(servoNormalize(servoHooked));
        hookUp = false;
    }
    public void hookOff(){
        hook.setPosition(servoNormalize(servoUnhooked));
        hookUp = true;
    }

    public void hookToggle(){
        if(hookUp)
            hookOn();
        else
            hookOff();
    }



    public void openGate(){
        intakeGate.setPosition(servoNormalize(servoGateOpen));
        gateOpen = true;
    }
    public void closeGate(){
        intakeGate.setPosition(servoNormalize(servoGateClosed));
        gateOpen = false;
    }

    public void gateToggle(){
        if(gateOpen)
            closeGate();
        else
            openGate();
    }



    public void collect(){
        intakeLeft.setPosition(.5 + intakePwr);
        intakeRight.setPosition(.5 + intakePwr);
    }
    public void eject(){
        intakeRight.setPosition(.5 - intakePwr);
        intakeLeft.setPosition(.5 - intakePwr);}
    public void stopIntake(){
        intakeRight.setPosition(.5);
        intakeLeft.setPosition(.5);
    }

    public boolean isActive(){
        return active;
    }

    public void setExtendABobTargetPos(int pos){
        extendABobPos = pos;
    }
    public int getExtendABobTargetPos(){
        return extendABobPos;
    }
    public int getExtendABobCurrentPos(){
        return extendABob.getCurrentPosition();
    }
    public void setExtendABobPwr(double pwr){ extendABobPwr = pwr; }

    public void setElbowTargetPos(int pos){
        elbowPos = pos;
    }
    public boolean setElbowTargetPos(int pos, double speed){
        setElbowTargetPos(pos);
        setElbowPwr(speed);
        if (nearTargetElbow()) return true;
        else return false;
    }
    public int getElbowTargetPos(){
        return elbowPos;
    }
    public int getElbowCurrentPos(){
        return elbow.getCurrentPosition();
    }
    public double getCurrentAngle(){return  elbow.getCurrentPosition()/ticksPerDegree;}
    public double getCurrentLength(){
        return (107.0/2960.0)*getExtendABobCurrentPos() + 46;
    }
    public void setElbowPwr(double pwr){ elbowPwr = pwr; }

    public void kill(){
        setElbowPwr(0);
        setExtendABobPwr(0);
        update();
        active = false;
    }
    public void restart(double elbowPwr, double extendABobPwr){
        setElbowPwr(elbowPwr);
        setExtendABobPwr(extendABobPwr);
        active = true;
    }

    public void resetEncoders() {
        //just encoders - only safe to call if we know collector is in normal starting position
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendABob.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendABob.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public boolean extendToMin(){
        return extendToMin(extendABobPwr, 15);
    }
    public boolean extendToMin(double speed, int range){
        setExtendABobPwr(speed);
        setExtendABobTargetPos(extendMin);
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<range){
            return true;
        }
        return false;
    }
    public boolean extendToLow(){
        return extendToLow(extendABobPwr, 15);
    }
    public boolean extendToLow(double speed, int range){
        setExtendABobPwr(speed);
        setExtendABobTargetPos(extendLow);
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<range){
            return true;
        }
        return false;
    }
    public boolean extendToMid(){
        return extendToMid(extendABobPwr, 15);
    }
    public boolean extendToMid(double speed, int range){
        setExtendABobPwr(speed);
        setExtendABobTargetPos(extendMid);
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<range){
            return true;
        }
        return false;
    }
    public boolean extendToPosition(int position, double speed, int range){
        setExtendABobPwr(speed);
        setExtendABobTargetPos(position);
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<range){
            return true;
        }
        return false;
    }
    public boolean extendToMax(){
        return extendToMax(extendABobPwr, 15);
    }
    public boolean extendToMax(double speed, int range){
        setExtendABobPwr(speed);
        setExtendABobTargetPos(extendMax);
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<range){
            return true;
        }
        return false;
    }

    public boolean extendToReverseDeposit(){
        return extendToMax(extendABobPwr, 15);
    }
    public boolean extendToReverseDeposit(double speed, int range){
        setExtendABobPwr(speed);
        setExtendABobTargetPos(extendDeposit);
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<range){
            return true;
        }
        return false;
    }

    public boolean nearTargetExtend(){
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<15) return true;
        else return false;
    }
    public boolean nearTargetElbow(){
        if ((Math.abs( getElbowCurrentPos()-getElbowTargetPos()))<15) return true;
        else return false;
    }
    public boolean nearTarget(){
        if (nearTargetElbow() && nearTargetExtend()) return true;
        else return false;
    }

    public void increaseElbowAngle(){
        setElbowTargetPos(Math.min(getElbowCurrentPos() + 100, pos_Intake));
    }
    public void decreaseElbowAngle(){
        setElbowTargetPos(Math.max(getElbowCurrentPos() - 100, 0));

    }

    public void extendBelt(){
        setExtendABobTargetPos(Math.min(getExtendABobCurrentPos() + 100, extendMax));
    }
    public void retractBelt(){
        setExtendABobTargetPos(Math.max(getExtendABobCurrentPos() - 100, extendMin));
    }

    public void runToAngle(double angle){
        setElbowTargetPos((int)(angle * ticksPerDegree));
    }//untested

    public static double servoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }
}
