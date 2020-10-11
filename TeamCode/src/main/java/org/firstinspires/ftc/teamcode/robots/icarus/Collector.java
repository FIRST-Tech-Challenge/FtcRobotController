package org.firstinspires.ftc.teamcode.robots.icarus;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 2938061 on 11/10/2017.
 */

public class Collector {

    //number of ticks per revolution REV HD motor: 2240
    //number of ticks per revolution REV Core    : 288

    DcMotor elbowLeft = null;
    DcMotor elbowRight = null;
    DcMotor extendABobLeft = null;
    DcMotor extendABobRight = null;

    Servo intakeRight = null;
    Servo intakeLeft = null;
    Servo hook = null;
    Servo intakeGate = null;

    int elbowPosInternal = 0;
    int elbowPos = 0;
    double elbowPwr = 0;

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

    //filler value; needs to be updated to reflect actual ratio
    public double ticksPerDegree = 22.3296703;

    public boolean active = true;

    public Collector(PoseBigWheel.RobotType robotType, DcMotor elbowLeft, DcMotor elbowRight, DcMotor extendABobLeft, DcMotor extendABobRight, Servo intakeRight, Servo intakeLeft, Servo hook, Servo intakeGate){

        elbowLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowRight.setDirection(DcMotorSimple.Direction.REVERSE);

        extendABobLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendABobRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendABobLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //extendABobRight.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeLeft.setDirection(Servo.Direction.REVERSE);

        this.elbowLeft = elbowLeft;
        this.elbowRight = elbowRight;
        this.extendABobLeft = extendABobLeft;
        this.extendABobRight = extendABobRight;
        this.intakeRight = intakeRight;
        this.intakeLeft = intakeLeft;
        this.hook = hook;
        this.intakeGate = intakeGate;


        switch (robotType){
            case BigWheel:
                intakePwr = .5;
                //normal Teleop encoder values
                pos_preIntake = 3600;
                pos_Intake   = 3900;
                pos_Deposit  = 1520;
                pos_reverseIntake = 80;
                pos_reversePreDeposit=1408;
                pos_reverseDeposit = 2908;
                pos_reverseSafeDrive = 1000;
                pos_PartialDeposit = 1700;
                pos_SafeDrive = 800;

                //autonomous encoder values
                pos_AutoPark = pos_SafeDrive + 500;
                pos_autonPrelatch = 2950;

                //end game encoder values
                pos_prelatch = 2558;
                pos_latched = 2700;
                pos_postlatch = 1240;

                servoGateOpen = 900;
                servoGateClosed = 1495;

                servoHooked = 1560;
                servoUnhooked = 1800;

                //belt extension encoder values
                extendMax = 2500;
                extendMid= 980;
                extendLow = 650; //clears hook and good for retracting prior to deposit without tipping robot
                extendMin = 300;  //prevent crunching collector tray
                break;
            case Icarus:
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

                servoGateOpen = 900;
                servoGateClosed = 1495;

                servoHooked = 1600;
                servoUnhooked = 1100;

                //belt extension encoder values
                extendDeposit = 1489;
                extendMax = 2960;
                extendMid= 980;
                extendLow = 650; //clears hook and good for retracting prior to deposit without tipping robot
                extendMin = 300;  //prevent crunching collector tray
                break;
        }

    }

    public void update(){
        if(active && elbowPosInternal!=elbowPos) { //don't keep updating if we are retractBelt to target position
            elbowPosInternal = elbowPos;
            elbowLeft.setTargetPosition(elbowPos);
            elbowRight.setTargetPosition(elbowPos);
            elbowLeft.setPower(elbowPwr);
            elbowRight.setPower(elbowPwr);
        }
        if(active && extendABobPosInternal!=extendABobPos) { //don't keep updating if we are retractBelt to target position
            extendABobPosInternal = extendABobPos;
            extendABobLeft.setTargetPosition(extendABobPos);
            extendABobRight.setTargetPosition(extendABobPos);
            extendABobLeft.setPower(extendABobPwr);
            extendABobRight.setPower(extendABobPwr);
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
    }
    public void hookOff(){
        hook.setPosition(servoNormalize(servoUnhooked));
    }

    public void openGate(){
        intakeGate.setPosition(servoNormalize(servoGateOpen));
    }
    public void closeGate(){
        intakeGate.setPosition(servoNormalize(servoGateClosed));
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
        return extendABobLeft.getCurrentPosition();
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
        return elbowLeft.getCurrentPosition();
    }
    public int getElbowCurrentPos2(){
        return elbowRight.getCurrentPosition();
    }
    public double getCurrentAngle(){return  elbowRight.getCurrentPosition()/ticksPerDegree;}
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
        elbowLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendABobLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendABobRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendABobRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendABobLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        setExtendABobTargetPos(Math.max(getExtendABobCurrentPos() - 250, extendMin));
    }

    public void extendBelt(){
        setExtendABobTargetPos(Math.min(getExtendABobCurrentPos() + 250, extendMax));
    }
    public void retractBelt(){
        setElbowTargetPos(Math.max(getElbowCurrentPos() - 250, 0));
    }

    public void runToAngle(double angle){
        setElbowTargetPos((int)(angle * ticksPerDegree));
    }//untested

    public static double servoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }
}
