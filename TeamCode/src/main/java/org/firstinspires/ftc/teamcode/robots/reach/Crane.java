package org.firstinspires.ftc.teamcode.robots.reach;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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

    public enum commonPositions{
        STARTING(0,0,0,0),
        HOME(0,0,0,0),
        LOWEST_TEIR(0,0,0,0),
        MIDDLE_TEIR(0,0,0,0),
        HIGH_TEIR(0,0,0,0),
        TRANSFER(0,0,0,0);


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

    commonPositions currentTargetPos = null;

    public commonPositions Do(commonPositions targetPos){
        currentTargetPos = targetPos;

        switch(currentTargetPos){
            case STARTING:
                setPos(commonPositions.STARTING);
            case HOME:
                //need to call method and then set targetPos to be 
            case TRANSFER:
            case HIGH_TEIR:
            case LOWEST_TEIR:
            case MIDDLE_TEIR:
            default:
        }

        return targetPos;
    }

    public void setPos(commonPositions currentTargetPos){

    }

    boolean initialized;

    public boolean commonTimer(int seconds){
        if(initialized){

        }
        return true;
    }

    public void resetCommonTimer(){
        initialized = false;
    }
}
