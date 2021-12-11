package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems.Turret;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants;

import java.util.Arrays;
import java.util.Map;

public class Crane implements Subsystem {

    Turret turret;
    Servo firstLinkServo;
    Servo secondLinkServo;
    Servo bucketServo;

    //use these maybe for motion smoothing
    int firstLinkServoTargetPos;
    int secondLinkServoTargetPos;
    int bucketServoTargetPos;
    double turretTargetPos;

    double toHomeTime = 2;//todo- update
    double avgTransferTime = 4; //todo- update
    int bucketUpPos = 900;
    int bucketDownPos = 1200;
    boolean isAtHome = false;

    public Crane(HardwareMap hardwareMap) {
        firstLinkServo = hardwareMap.get(Servo.class, "firstLinkServo");
        secondLinkServo = hardwareMap.get(Servo.class, "secondLinkServo");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");

        turret = new Turret(hardwareMap);
    }

    public enum CommonPosition {
        STARTING(0,0,0,0),
        HOME(0,0,0,0),
        LOWEST_TEIR(0,0,0,0),
        MIDDLE_TEIR(0,0,0,0),
        HIGH_TEIR(0,0,0,0),
        TRANSFER(0,0,0,0),
        FINISHED(0,0,0,0);


        public int firstLinkPos, secondLinkPos, bucketServoPos;
        public double turretAngle;

        CommonPosition(int firstLinkPos, int secondLinkPos, int bucketServoPos, double turretAngle){
            this.firstLinkPos = firstLinkPos;
            this.secondLinkPos = secondLinkPos;
            this.bucketServoPos = bucketServoPos;
            this.turretAngle = turretAngle;
        }
    }

    CommonPosition currentTargetPos = CommonPosition.FINISHED;

    public CommonPosition Do(CommonPosition targetPos) {
        currentTargetPos = targetPos;


        switch(currentTargetPos){
            case STARTING:
                setPos(CommonPosition.STARTING);
                break;
            case HOME:
                setPosSafeley(CommonPosition.HOME);
                break;
            case TRANSFER:
                setPosSafeley(CommonPosition.TRANSFER);
                break;
            case HIGH_TEIR:
                setPosSafeley(CommonPosition.HIGH_TEIR);
                break;
            case LOWEST_TEIR:
                setPosSafeley(CommonPosition.LOWEST_TEIR);
                break;
            case MIDDLE_TEIR:
                setPosSafeley(CommonPosition.MIDDLE_TEIR);
                break;
            default:
                break;
        }

        return currentTargetPos;
    }

    public boolean doAuton(CommonPosition targetPos){
        if(Do(targetPos) == CommonPosition.FINISHED){
            return true;
        }
        return true;
    }

    private boolean checkForHome(){
        if(isAtHome){
            setPos(CommonPosition.HOME);
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

    private void setPosSafeley(CommonPosition targetPos){
        if(checkForHome()){

            setPos(targetPos);

            if(commonTimer(avgTransferTime)){
                currentTargetPos = CommonPosition.FINISHED;
                isAtHome = (targetPos == CommonPosition.HOME);
            }
        }
    }

    private void setPos(CommonPosition targetPos){
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

    @Override
    public void update(){
        turret.setTurretAngle(turretTargetPos);
        firstLinkServo.setPosition(firstLinkServoTargetPos);
        secondLinkServo.setPosition(secondLinkServoTargetPos);
        bucketServo.setPosition(bucketServoTargetPos);

        Do(currentTargetPos);
        turret.update();
    }

    @Override
    public void stop(){

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        return null;
    }

    @Override
    public String getTelemetryName() {
        return null;
    }
}

