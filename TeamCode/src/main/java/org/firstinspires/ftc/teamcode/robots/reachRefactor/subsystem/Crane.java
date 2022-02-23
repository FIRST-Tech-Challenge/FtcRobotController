package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.LinkedHashMap;
import java.util.Map;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.DistanceSensorSim;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.ServoSim;
import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Utils.*;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

@Config
public class Crane implements Subsystem {
    public static int SHOULDER_HOME_PWM = 1550;
    public static int ELBOW_HOME_PWM = 1500;
    public static int WRIST_HOME_PWM = 1500;

    public static double SHOULDER_PWM_PER_DEGREE = 600.0 / 90.0;
    public static double ELBOW_PWM_PER_DEGREE = -600.0 / 90.0;
    public static double WRIST_PWM_PER_DEGREE = 750.0 / 180.0;

    public static double SHOULDER_DEG_MIN = -90; //negative angles are counter clockwise while looking at the left side of the robot
    public static double ELBOW_DEG_MIN = -60;
    public static double WRIST_DEG_MIN = -180;

    public static double SHOULDER_DEG_MAX = 90;
    public static double ELBOW_DEG_MAX = 140;
    public static double WRIST_DEG_MAX = 180;

    public Turret turret;

    public Servo shoulderServo, elbowServo, wristServo;
    private final DistanceSensor bucketDistanceSensor;

    private double bucketDistance;
    private double shoulderTargetAngle, elbowTargetAngle, wristTargetAngle;
    private boolean dumping;

    private Articulation articulation;

    public Crane(HardwareMap hardwareMap, Turret turret, boolean simulated) {
        if(simulated) {
            shoulderServo = new ServoSim();
            elbowServo = new ServoSim();
            wristServo = new ServoSim();
            bucketDistanceSensor = new DistanceSensorSim(100);
        } else {
            shoulderServo = hardwareMap.get(Servo.class, "firstLinkServo");
            elbowServo = hardwareMap.get(Servo.class, "secondLinkServo");
            wristServo = hardwareMap.get(Servo.class, "bucketServo");
            bucketDistanceSensor = hardwareMap.get(DistanceSensor.class, "distBucket");
        }

        this.turret = turret;
        articulation = Articulation.MANUAL;
    }

    public enum Articulation {
        TEST_INIT(0, 0, 0, 0, 5,0),
        MANUAL(0, 0, 0, 0, 0,0),

        INIT(-90,0,90,0, 1.5f,90),
        HOME(0,0,0,0, 0,0),
      
        LOWEST_TIER(75,130,20, 1.5f, 130),
        MIDDLE_TIER(60,130,40, 1f, 150),
        HIGH_TIER(22, 125,70, 1f, 170),
        HIGH_TIER_LEFT(20, 125,70,-80, 1f, 170),
        HIGH_TIER_RIGHT(20, 125,70,80, 1f, 170),
        TRANSFER(-45,-50,-20,0, 0.75f,0),

        CAP(30, 140,0,0, 1, 170),
      
        //these articulations are meant to observe the motions and angles to check for belt skips
        VALIDATE_ELBOW90(0,90,90,0, .5f,0),
        VALIDATE_SHOULDER90(90,15,-90+15,0, .5f,0),
        VALIDATE_TURRET90R(0,0,0,45,2.5f,0),
        VALIDATE_TURRET90L(0,0,0,-45,2.5f,0),

        //auton articulations
        AUTON_REACH_RIGHT(40, 130,70,30, 1, 170),
        AUTON_REACH_LEFT(40, 130,70,-30, 1, 170);

        public int shoulderPos, elbowPos, wristPos;
        public double turretAngle;
        public float toHomeTime;
        public int dumpPos;
        public boolean turret;

        Articulation(int shoulderPos, int elbowPos, int wristPos, double turretAngle, float toHomeTime, int dumpPos){
            this.shoulderPos = shoulderPos;
            this.elbowPos = elbowPos;
            this.wristPos = wristPos;
            this.turretAngle = turretAngle;
            this.toHomeTime = toHomeTime;
            this.dumpPos = dumpPos;
            turret = true;
        }

        Articulation(int shoulderPos, int elbowPos, int wristPos, float toHomeTime, int dumpPos){
            this.shoulderPos = shoulderPos;
            this.elbowPos = elbowPos;
            this.wristPos = wristPos;
            this.toHomeTime = toHomeTime;
            this.dumpPos = dumpPos;
            turret = false;
        }
    }

    private float currentToHomeTime = Articulation.HOME.toHomeTime;
    private double currentDumpPos = 0;
    private final Stage mainStage = new Stage();
    private final StateMachine main = getStateMachine(mainStage)
            .addSingleState(() -> { dumping = false; })
            .addTimedState(() -> currentToHomeTime, () -> setTargetPositions(Articulation.HOME), () -> {})
            .addTimedState(() -> articulation.toHomeTime, () -> setTargetPositions(articulation),
                    () -> {
                        currentToHomeTime = articulation.toHomeTime;
                        if(articulation.dumpPos!=0) currentDumpPos = articulation.dumpPos;
                    }
            )

            .build();

    private final Stage initStage = new Stage();
    private final StateMachine init = getStateMachine(initStage)
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
    public void update(Canvas fieldOverlay){
        bucketDistance = bucketDistanceSensor.getDistance(DistanceUnit.INCH);

        articulate(articulation);

        if(shoulderTargetAngle > 180)
            shoulderTargetAngle -= 360;
        if(elbowTargetAngle > 180)
            elbowTargetAngle -= 360;
        if(wristTargetAngle > 180)
            wristTargetAngle -= 360;

        shoulderServo.setPosition(servoNormalize(shoulderServoValue(shoulderTargetAngle)));
        elbowServo.setPosition(servoNormalize(elbowServoValue(elbowTargetAngle)));
        wristServo.setPosition(servoNormalize(wristServoValue(wristTargetAngle)));

        if(articulation != Articulation.MANUAL)
            turret.setTargetHeading(articulation.turretAngle);
        turret.update(fieldOverlay);
    }

    @Override
    public void stop() {
        articulation = Articulation.HOME;
    }

    @Override
    public String getTelemetryName() {
        return "Crane";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();

        telemetryMap.put("Current Articulation", articulation);

        if(debug) {
            telemetryMap.put("Shoulder Target Angle", shoulderTargetAngle);
            telemetryMap.put("Elbow Target Angle", elbowTargetAngle);
            telemetryMap.put("Wrist Target Angle", wristTargetAngle);

            telemetryMap.put("Shoulder Target PWM", shoulderServoValue(shoulderTargetAngle));
            telemetryMap.put("Elbow Target PWM", elbowServoValue(elbowTargetAngle));
            telemetryMap.put("Wrist Target PWM", wristServoValue(wristTargetAngle));

            telemetryMap.put("bucket distance", bucketDistance);
        }

        telemetryMap.put("Turret:", "");
        Map<String, Object> turretTelemetryMap = turret.getTelemetry(debug);
        telemetryMap.putAll(turretTelemetryMap);

        return telemetryMap;
    }


    public void dump() {
        setWristTargetAngle(currentDumpPos);
        dumping = true;
    }

    private void setTargetPositions(Articulation articulation) {
        setShoulderTargetAngle(articulation.shoulderPos);
        setElbowTargetAngle(articulation.elbowPos);
        setWristTargetAngle(articulation.wristPos);

        if(articulation.turret)
            turret.setTargetHeading(articulation.turretAngle);
    }

    //----------------------------------------------------------------------------------------------
    // Getters And Setters
    //----------------------------------------------------------------------------------------------

    //take the supplied relative-to-home target value in degrees
    //and convert to servo setting
    private double shoulderServoValue(double targetPos){
        double newPos = Range.clip(targetPos,SHOULDER_DEG_MIN, SHOULDER_DEG_MAX);
        newPos = newPos * SHOULDER_PWM_PER_DEGREE + SHOULDER_HOME_PWM;
        return newPos;
    }

    private double elbowServoValue(double targetPos){
        double newPos = Range.clip(targetPos,ELBOW_DEG_MIN, ELBOW_DEG_MAX);
        newPos = newPos * ELBOW_PWM_PER_DEGREE + ELBOW_HOME_PWM;
        return newPos;
    }

    private double wristServoValue(double targetPos){
        double newPos = Range.clip(targetPos,WRIST_DEG_MIN, WRIST_DEG_MAX);
        newPos = newPos * WRIST_PWM_PER_DEGREE + WRIST_HOME_PWM;
        return newPos;
    }

    public void setShoulderTargetAngle(double shoulderTargetAngle) {
        this.shoulderTargetAngle = wrapAngle(shoulderTargetAngle);
    }

    public void setElbowTargetAngle(double elbowTargetAngle) {
        this.elbowTargetAngle = wrapAngle(elbowTargetAngle);
    }

    public void setWristTargetAngle(double wristTargetAngle) {
        this.wristTargetAngle = wrapAngle(wristTargetAngle);
    }

    public void setDumpPos(double dumpPos) {
        this.currentDumpPos = wrapAngle(dumpPos);
    }

    public double getShoulderTargetAngle() {
        return shoulderTargetAngle;
    }

    public double getElbowTargetAngle() {
        return elbowTargetAngle;
    }

    public double getWristTargetAngle() {
        return wristTargetAngle;
    }

    public Articulation getArticulation() { return articulation; }

    public boolean isDumping() { return dumping; }

    public double getBucketDistance() {
        return bucketDistance;
    }
}

