package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.HashMap;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.UtilMethods.servoNormalize;

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

    private Articulation previousArticulation, articulation;

    // Constants
    private static final String TELEMETRY_NAME = "Crane";

    public static int BUCKET_UP_POS = 900;
    public static int BUCKET_DOWN_POS = 1200;

    public Crane(HardwareMap hardwareMap) {
        shoulderServo = hardwareMap.get(ServoImplEx.class, "firstLinkServo");
        elbowServo = hardwareMap.get(ServoImplEx.class, "secondLinkServo");
        wristServo = hardwareMap.get(ServoImplEx.class, "bucketServo");

        turret = new Turret(hardwareMap);
        articulation = Articulation.MANUAL;
        previousArticulation = Articulation.MANUAL;
    }

    public enum Articulation {
        INIT(2169, 1183, 1316, 0, 5),
        MANUAL(0, 0, 0, 0, 0),
        STARTING(2200,1600,1600,0, 5),
        HOME(1700,1650,1600,0, 0),
        LOWEST_TEIR(0,0,0,0, 5),
        MIDDLE_TEIR(0,0,0,0, 5),
        HIGH_TEIR(1500, 1000,1650,0, 5),
        CAP(1500, 900,1650,0, 5),
        TRANSFER(1850,2000,1550,0, 5);

        public int shoulderPos, elbowPos, wristPos;
        public double turretAngle;
        public int toHomeTime;

        Articulation(int shoulderPos, int elbowPos, int wristPos, double turretAngle, int toHomeTime){
            this.shoulderPos = shoulderPos;
            this.elbowPos = elbowPos;
            this.wristPos = wristPos;
            this.turretAngle = turretAngle;
            this.toHomeTime = toHomeTime;
        }
    }

    private Stage mainStage = new Stage();
    private StateMachine main = UtilMethods.getStateMachine(mainStage)
            .addTimedState(() -> previousArticulation.toHomeTime, () -> setTargetPositions(Articulation.HOME), () -> {})
            .addTimedState(() -> articulation.toHomeTime, () -> setTargetPositions(articulation), () -> {})
            .build();

    private Stage initStage = new Stage();
    private StateMachine init = UtilMethods.getStateMachine(initStage)
            .addTimedState(2f, () -> setTargetPositions(Articulation.INIT), () -> {})
            .build();

    public boolean articulate(Articulation articulation) {
        if(articulation.equals(Articulation.MANUAL))
            return true;
        else if(articulation.equals(Articulation.INIT)) {
            this.articulation = articulation;
            return init.execute();
        } else {
            previousArticulation = this.articulation;
            this.articulation = articulation;
            return main.execute();
        }
    }

    public boolean flipBucket(boolean down) {
        wristTargetPos = down ? BUCKET_DOWN_POS : BUCKET_UP_POS;
        return true;
    }

    @Override
    public void update(){
        articulate(articulation);

        shoulderServo.setPosition(servoNormalize(shoulderTargetPos));
        elbowServo.setPosition(servoNormalize(elbowTargetPos));
        wristServo.setPosition(servoNormalize(wristTargetPos));

        turret.update();
    }

    @Override
    public void stop(){
        turret.stop();
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new HashMap<String, Object>();

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

    private void setTargetPositions(Articulation articulation) {
        this.shoulderTargetPos = articulation.shoulderPos;
        this.elbowTargetPos = articulation.elbowPos;
        this.wristTargetPos = articulation.wristPos;

        turret.setTargetAngle(articulation.turretAngle);
    }

    //----------------------------------------------------------------------------------------------
    // Getters And Setters
    //----------------------------------------------------------------------------------------------


    public void setShoulderTargetPos(int shoulderTargetPos) {
        this.shoulderTargetPos = shoulderTargetPos;
    }

    public void setElbowTargetPos(int elbowTargetPos) {
        this.elbowTargetPos = elbowTargetPos;
    }

    public void setWristTargetPos(int wristTargetPos) {
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

