package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import java.util.HashMap;
import java.util.Map;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.UtilMethods;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

@Config
public class Crane implements Subsystem {

    public Turret turret;

    // Servos
    public ServoImplEx shoulderServo, elbowServo, wristServo;

    // State
    private int shoulderTargetPos, elbowTargetPos, wristTargetPos;
    private double turretTargetPos;

    private Articulation articulation;

    // Constants
    private static final String TELEMETRY_NAME = "Crane";

    public static int BUCKET_UP_POS = 900;
    public static int BUCKET_DOWN_POS = 1200;

    public static int SHOULDER_HOME_PWM = 1550;
    public static int ELBOW_HOME_PWM = 1550;
    public static int WRIST_HOME_PWM = 1500;

    public static double SHOULDER_PWM_PER_DEGREE = -600/90;
    public static double ELBOW_PWM_PER_DEGREE = -600/90;
    public static double WRIST_PWM_PER_DEGREE = 750/180;

    public static double SHOULDER_DEG_MIN = -90; //negative angles are counter clockwise while looking at the left side of the robot
    public static double ELBOW_DEG_MIN = -40;
    public static double WRIST_DEG_MIN = -180;

    public static double SHOULDER_DEG_MAX = 90;
    public static double ELBOW_DEG_MAX = 140;
    public static double WRIST_DEG_MAX = 180;

    // transfer
    // shoulder: 1933,
    // elbow: 1879,
    // wrist: 1496,




    public Crane(HardwareMap hardwareMap, Turret turret) {
        shoulderServo = hardwareMap.get(ServoImplEx.class, "firstLinkServo");
        elbowServo = hardwareMap.get(ServoImplEx.class, "secondLinkServo");
        wristServo = hardwareMap.get(ServoImplEx.class, "bucketServo");

        this.turret = turret;
        articulation = Articulation.MANUAL;
    }

    public enum Articulation {
        INIT(0, 0, 0, 0, 5,0),
        MANUAL(0, 0, 0, 0, 0,0),
        SIZING(-90,0,70,0, 1.5f,0),
        HOME(0,0,0,0, 0,0),
        LOWEST_TIER(1043,975,1556,0, 1.5f, 130, true),
        MIDDLE_TIER(1273,1233,1621,0, 1, 150, true),
        HIGH_TIER(1391, 1128,1751,0, 1, 170, true),
        CAP(30, 140,0,0, 1, 170),
        TRANSFER(1933,1879,1496,0, 2,0, true),
        //these articulations are meant to observe the motions and angles to check for belt skips
        VALIDATE_ELBOW90(0,90,90,0, .5f,0),
        VALIDATE_SHOULDER90(90,15,-90+15,0, .5f,0),
        VALIDATE_TURRET90R(0,0,0,45,2.5f,0),
        VALIDATE_TURRET90L(0,0,0,-45,2.5f,0);


        public int shoulderPos, elbowPos, wristPos;
        public double turretAngle;
        public float toHomeTime;
        public int dumpPos;
        public boolean rawValues;

        Articulation(int shoulderPos, int elbowPos, int wristPos, double turretAngle, float toHomeTime, int dumpPos){
            this.shoulderPos = shoulderPos;
            this.elbowPos = elbowPos;
            this.wristPos = wristPos;
            this.turretAngle = turretAngle;
            this.toHomeTime = toHomeTime;
            this.dumpPos = dumpPos;
        }

        Articulation(int shoulderPos, int elbowPos, int wristPos, double turretAngle, float toHomeTime, int dumpPos, boolean rawValues) {
            this(shoulderPos, elbowPos, wristPos, turretAngle, toHomeTime, dumpPos);
            this.rawValues = rawValues;
        }
    }

    private float currentToHomeTime = Articulation.HOME.toHomeTime;
    private int currentDumpPos = 0;
    private final Stage mainStage = new Stage();
    private final StateMachine main = UtilMethods.getStateMachine(mainStage)
            .addTimedState(() -> currentToHomeTime, () -> setTargetPositions(Articulation.HOME), () -> {})
            .addTimedState(() -> 0, () -> setTargetPositions(articulation),
                    () -> {
                        currentToHomeTime = articulation.toHomeTime;
                        if(articulation.dumpPos!=0) currentDumpPos= articulation.dumpPos;}
                        )

            .build();

    private final Stage initStage = new Stage();
    private final StateMachine init = UtilMethods.getStateMachine(initStage)
            .addTimedState(2f, () -> setTargetPositions(Articulation.INIT), () -> {})
            .build();

    public boolean articulate(Articulation articulation) {
        if(articulation.equals(Articulation.MANUAL))
            return true;
        else if(articulation.equals(Articulation.INIT)) {
            this.articulation = articulation;
            if(init.execute()) {
                this.articulation = Articulation.MANUAL;
                return true;
            }
        }
        else {
            this.articulation = articulation;
            if(main.execute()) {
                this.articulation = Articulation.MANUAL;
                return true;
            }
        }
        return false;
    }

    @Override
    public void update(){
        articulate(articulation);

        shoulderServo.setPosition(UtilMethods.servoNormalize(shoulderTargetPos));
        elbowServo.setPosition(UtilMethods.servoNormalize(elbowTargetPos));
        wristServo.setPosition(UtilMethods.servoNormalize(wristTargetPos));
        turret.setTargetAngle(turretTargetPos);
    }

    //take the supplied relative-to-home target value in degrees
    //and convert to servo setting
    private double shoulderServoValue(double targetpos){
        double newpos = Range.clip(targetpos,SHOULDER_DEG_MIN, SHOULDER_DEG_MAX);
        newpos = newpos * SHOULDER_PWM_PER_DEGREE+SHOULDER_HOME_PWM;
        return newpos;
    }

    private double elbowServoValue(double targetpos){
        double newpos = Range.clip(targetpos,ELBOW_DEG_MIN, ELBOW_DEG_MAX);
        newpos = newpos*ELBOW_PWM_PER_DEGREE+ELBOW_HOME_PWM;
        return newpos;
    }

    private double wristServoValue(double targetpos){
        double newpos = Range.clip(targetpos,WRIST_DEG_MIN, WRIST_DEG_MAX);
        newpos = newpos*WRIST_PWM_PER_DEGREE+WRIST_HOME_PWM;
        return newpos;
    }

    private double shoulderPositionToAngle(double targetAngle) {
        double newpos = Range.clip(targetAngle,SHOULDER_DEG_MIN, SHOULDER_DEG_MAX);
        newpos = (newpos - SHOULDER_HOME_PWM) / SHOULDER_PWM_PER_DEGREE;
        return newpos;
    }


    @Override
    public void stop(){
        turret.stop();
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new HashMap<>();

        telemetryMap.put("Current Articulation", articulation);

        if(debug) {
            telemetryMap.put("Shoulder Target Position", shoulderTargetPos);
            telemetryMap.put("Elbow Target Position", elbowTargetPos);
            telemetryMap.put("Wrist Target Position", wristTargetPos);
        }

        telemetryMap.put("Turret:", "");
        Map<String, Object> turretTelemetryMap = turret.getTelemetry(debug);
        telemetryMap.putAll(turretTelemetryMap);

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return TELEMETRY_NAME;
    }

    public void Dump(){
        setWristTargetPos(currentDumpPos);
    }

    private void setTargetPositions(Articulation articulation) {
        if(articulation.rawValues) {
            this.shoulderTargetPos = articulation.shoulderPos;
            this.elbowTargetPos = articulation.elbowPos;
            this.wristTargetPos = articulation.wristPos;
        } else {
            setShoulderTargetPos(articulation.shoulderPos);
            setElbowTargetPos(articulation.elbowPos);
            setWristTargetPos(articulation.wristPos);
        }

        this.turretTargetPos = articulation.turretAngle;

        turret.setTargetAngle(articulation.turretAngle);
    }

    //----------------------------------------------------------------------------------------------
    // Getters And Setters
    //----------------------------------------------------------------------------------------------


    public void setShoulderTargetPos(int shoulderTargetPos) {
        this.shoulderTargetPos = (int) shoulderServoValue(shoulderTargetPos);
    }

    public void setElbowTargetPos(int elbowTargetPos) {
        this.elbowTargetPos = (int) elbowServoValue(elbowTargetPos);
    }

    public void setWristTargetPos(int wristTargetPos) {
        this.wristTargetPos = (int) wristServoValue(wristTargetPos);
    }

    public void setShoulderTargetPosRaw(int shoulderTargetPos) {
        this.shoulderTargetPos = shoulderTargetPos;
    }

    public void setElbowTargetPosRaw(int elbowTargetPos) {
        this.elbowTargetPos = elbowTargetPos;
    }

    public void setWristTargetPosRaw(int wristTargetPos) {
        this.wristTargetPos = wristTargetPos;
    }

    public int getShoulderTargetPos() {
        return shoulderTargetPos;
    }

    public int getElbowTargetPos() {
        return elbowTargetPos;
    }

    public int getWristTargetPos() {
        return wristTargetPos;
    }
}

