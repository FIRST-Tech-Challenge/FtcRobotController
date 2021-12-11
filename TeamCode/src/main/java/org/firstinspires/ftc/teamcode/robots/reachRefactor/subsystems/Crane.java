package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems.Turret;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.UtilMethods;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class Crane implements Subsystem {

    public Turret turret;
    private Servo firstLinkServo;
    private Servo secondLinkServo;
    private Servo bucketServo;

    //use these maybe for motion smoothing
    private int firstLinkServoTargetPos;
    private int secondLinkServoTargetPos;
    private int bucketServoTargetPos;
    private double turretTargetPos;

    private double toHomeTime = 2;//todo- update
    private double avgTransferTime = 4; //todo- update
    private int bucketUpPos = 900;
    private int bucketDownPos = 1200;
    private boolean isAtHome = false;

    private static final String TELEMETRY_NAME = "Crane";

    public Crane(HardwareMap hardwareMap) {
        firstLinkServo = hardwareMap.get(Servo.class, "firstLinkServo");
        secondLinkServo = hardwareMap.get(Servo.class, "secondLinkServo");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");

        turret = new Turret(hardwareMap);
    }

    public enum CommonPosition {
        STARTING(2100,1650,1600,0),
        HOME(1700,1650,1600,0),
        LOWEST_TEIR(0,0,0,0),
        MIDDLE_TEIR(0,0,0,0),
        HIGH_TEIR(1500, 1000,1650,0),
        CAP(1500, 900,1650,0),
        TRANSFER(1850,2000,1550,0),
        FINISHED(0,0,0,0);


        public int firstLinkPos, secondLinkPos, bucketServoPos;
        public double turretAngle;

        CommonPosition(int firstLinkPos, int secondLinkPos, int bucketServoPos, double turretAngle){
            this.firstLinkPos = (int) UtilMethods.servoNormalize(firstLinkPos);
            this.secondLinkPos = (int) UtilMethods.servoNormalize(secondLinkPos);
            this.bucketServoPos = (int) UtilMethods.servoNormalize(bucketServoPos);
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
        secondLinkServo.setPosition(targetPos.secondLinkPos);
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
        Map<String, Object> telemetryMap = new HashMap<String, Object>();

        telemetryMap.put("Turret:", "");
        Map<String, Object> turretTelemetryMap = turret.getTelemetry(debug);
        telemetryMap.putAll(turretTelemetryMap);

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return TELEMETRY_NAME;
    }
}

