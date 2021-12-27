package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

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

    public static int SHOULDER_HOME_PWM = 1500;
    public static int ELBOW_HOME_PWM = 1500;
    public static int WRIST_HOME_PWM = 1500;

    public static double SHOULDER_PWM_PER_DEGREE = 750/90;
    public static double ELBOW_PWM_PER_DEGREE = 750/90;
    public static double WRIST_PWM_PER_DEGREE = 750/180;

    public static double SHOULDER_DEG_MIN = -90; //negative angles are counter clockwise while looking at the left side of the robot
    public static double ELBOW_DEG_MIN = -40;
    public static double WRIST_DEG_MIN = -180;

    public static double SHOULDER_DEG_MAX = 90;
    public static double ELBOW_DEG_MAX = 140;
    public static double WRIST_DEG_MAX = 180;


    public Crane(HardwareMap hardwareMap) {
        shoulderServo = hardwareMap.get(ServoImplEx.class, "firstLinkServo");
        elbowServo = hardwareMap.get(ServoImplEx.class, "secondLinkServo");
        wristServo = hardwareMap.get(ServoImplEx.class, "bucketServo");

        turret = new Turret(hardwareMap);
        articulation = Articulation.MANUAL;
        previousArticulation = Articulation.MANUAL;
    }

    public enum Articulation {
        INIT(0, 0, 0, 0, 5),
        MANUAL(0, 0, 0, 0, 0),
        STARTING(-90,0,90,0, 5),
        HOME(0,0,0,0, 0),
        LOWEST_TIER(90,130,0,0, 5),
        MIDDLE_TIER(70,130,0,0, 5),
        HIGH_TIER(30, 130,0,0, 5),
        CAP(30, 140,0,0, 5),
        TRANSFER(-20,-40,-40,0, 5);

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

    private final Stage mainStage = new Stage();
    private final StateMachine main = UtilMethods.getStateMachine(mainStage)
            .addTimedState(() -> previousArticulation.toHomeTime, () -> setTargetPositions(Articulation.HOME), () -> {})
            .addTimedState(() -> articulation.toHomeTime, () -> setTargetPositions(articulation), () -> {})
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
        } else {
            previousArticulation = this.articulation;
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

        shoulderServo.setPosition(ShoulderServoValue(shoulderTargetPos));
        elbowServo.setPosition(ElbowServoValue(elbowTargetPos));
        wristServo.setPosition(WristServoValue(wristTargetPos));

        turret.update();
    }

    //take the supplied relative-to-home target value in degrees
    //and convert to servo setting
    private double ShoulderServoValue(double targetpos){
        double newpos = Range.clip(targetpos,SHOULDER_DEG_MIN, SHOULDER_DEG_MAX);
        newpos = newpos*SHOULDER_PWM_PER_DEGREE+SHOULDER_HOME_PWM;
        return servoNormalize(newpos);
    }

    private double ElbowServoValue(double targetpos){
        double newpos = Range.clip(targetpos,ELBOW_DEG_MIN, ELBOW_DEG_MAX);
        newpos = newpos*ELBOW_PWM_PER_DEGREE+ELBOW_HOME_PWM;
        return servoNormalize(newpos);
    }

    private double WristServoValue(double targetpos){
        double newpos = Range.clip(targetpos,WRIST_DEG_MIN, WRIST_DEG_MAX);
        newpos = newpos*WRIST_PWM_PER_DEGREE+WRIST_HOME_PWM;
        return servoNormalize(newpos);
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

