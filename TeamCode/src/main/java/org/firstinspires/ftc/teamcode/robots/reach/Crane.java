package org.firstinspires.ftc.teamcode.robots.reach;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.UGBot.utils.Constants;

public class Crane {

    Turret turret;
    Servo firstLinkServo;
    Servo secondLinkServo;
    Servo bucketServo;

    //use these maybe for motion smoothing
    int firstLinkServoTargetPos;
    int secondLinkServoTargetPos;
    int bucketServoTargetPos;
    double turretTargetPos;

    final double toHomeTime = 2;//todo- update
    final double avgTransferTime = 4; //todo- update
    final int bucketUpPos = 900;
    final int bucketDownPos = 1200;
    boolean isAtHome = false;

    public enum commonPositions{
        STARTING(0,0,0,0),
        HOME(0,0,0,0),
        LOWEST_TEIR(0,0,0,0),
        MIDDLE_TEIR(0,0,0,0),
        HIGH_TEIR(0,0,0,0),
        TRANSFER(0,0,0,0),
        FINISHED(0,0,0,0);


        public int firstLinkPos, secondLinkPos, bucketServoPos;
        public double turretAngle;

            private commonPositions(int firstLinkPos, int secondLinkPos, int bucketServoPos, double turretAngle){
                this.firstLinkPos = firstLinkPos;
                this.secondLinkPos = secondLinkPos;
                this.bucketServoPos = bucketServoPos;
                this.turretAngle = turretAngle;
            }
        }

    public Crane(DcMotor turretMotor, Servo firstLinkServo, Servo secondLinkServo, Servo bucketServo){
        turret = new Turret(turretMotor);
        this.firstLinkServo = firstLinkServo;
        this.secondLinkServo = secondLinkServo;
        this.bucketServo = bucketServo;
    }

    commonPositions currentTargetPos = commonPositions.FINISHED;

    public commonPositions Do(commonPositions targetPos){
        currentTargetPos = targetPos;


        switch(currentTargetPos){
            case STARTING:
                setPos(commonPositions.STARTING);
                break;
            case HOME:
                setPosSafeley(commonPositions.HOME);
                break;
            case TRANSFER:
                setPosSafeley(commonPositions.TRANSFER);
                break;
            case HIGH_TEIR:
                setPosSafeley(commonPositions.HIGH_TEIR);
                break;
            case LOWEST_TEIR:
                setPosSafeley(commonPositions.LOWEST_TEIR);
                break;
            case MIDDLE_TEIR:
                setPosSafeley(commonPositions.MIDDLE_TEIR);
                break;
            default:
                break;
        }

        return targetPos;
    }

    public boolean doAuton(commonPositions targetPos){
        if(Do(targetPos) == commonPositions.FINISHED){
            return true;
        }
        return true;
    }

    private boolean checkForHome(){
        if(isAtHome){
            setPos(commonPositions.HOME);
            if(commonTimer(toHomeTime)){
                isAtHome = true;
                return true;
            }
        }
        else {
            return true;
        }
        return false;
    }

    private void setPosSafeley(commonPositions targetPos){
        if(checkForHome()){

            setPos(targetPos);

            if(commonTimer(avgTransferTime)){
                currentTargetPos = commonPositions.FINISHED;
                isAtHome = (targetPos == commonPositions.HOME);
            }
        }
    }

    private void setPos(commonPositions targetPos){
        turret.setTurretAngle(targetPos.turretAngle);
        firstLinkServo.setPosition(targetPos.firstLinkPos);
        firstLinkServo.setPosition(targetPos.firstLinkPos);
        bucketServo.setPosition(targetPos.bucketServoPos);
    }

    boolean initialized;
    double commonTimerStartTime = 0;
    private boolean commonTimer(double seconds){
        if(initialized){
            commonTimerStartTime = System.nanoTime();
        }

        if(System.nanoTime() - commonTimerStartTime > (seconds * 1E9)){
            initialized = false;
            return true;
        }

        return false;
    }

    public boolean flipBucket(boolean down){
        bucketServoTargetPos = (down) ? bucketDownPos : bucketUpPos;
        return true;
    }

    public void update(){
        turret.setTurretAngle(turretTargetPos);
        firstLinkServo.setPosition(firstLinkServoTargetPos);
        secondLinkServo.setPosition(secondLinkServoTargetPos);
        bucketServo.setPosition(bucketServoTargetPos);

        Do(currentTargetPos);
    }
}
