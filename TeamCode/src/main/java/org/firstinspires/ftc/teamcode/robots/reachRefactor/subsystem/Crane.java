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

import org.firstinspires.ftc.teamcode.robots.reachRefactor.util.ExponentialSmoother;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

@Config(value = "FFCrane")
public class Crane implements Subsystem {
    public static int SHOULDER_HOME_PWM = 1550;
    public static int ELBOW_HOME_PWM = 1500;
    public static int WRIST_HOME_PWM = 1500;

    public static double SHOULDER_PWM_PER_DEGREE = 600.0 / 90.0;
    public static double ELBOW_PWM_PER_DEGREE = -600.0 / 90.0;
    public static double WRIST_PWM_PER_DEGREE = 750.0 / 180.0;

    public static double SHOULDER_DEG_MIN = -90; // negative angles are counter clockwise while looking at the left side
                                                 // of the robot
    public static double ELBOW_DEG_MIN = -80;
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
    private boolean toHomeEnabled;

    private Articulation articulation;

    public Crane(HardwareMap hardwareMap, Turret turret, boolean simulated) {
        if (simulated) {
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
        toHomeEnabled = true;
    }

    public enum Articulation {
        TEST_INIT(0, 0, 0, 0, 5, 0),
        MANUAL(0, 0, 0, 0, 0, 0),

        INIT(-90, 0, 90, 0, 1.5f, 90),
        HOME(0, 0, 0, 0, 0, 0),

        LOWEST_TIER(75, 130, 20, 1.5f, 130),
        MIDDLE_TIER(60, 130, 40, 1f, 150),
        HIGH_TIER(14.57741692662239, 113, 50.37986606359482, 1f, 170),
        HIGH_TIER_LEFT(14.57741692662239, 113, 50.37986606359482, -90, 1f, 180),
        HIGH_TIER_RIGHT(14.57741692662239, 113, 50.37986606359482, 90, 1f, 170),
        TRANSFER(-45, -50, -20, 0, 0.4f, 0),

        AUTON_LOWEST_TIER(45.957, 47.5, 44.253, 1.5f, 110),
        AUTON_MIDDLE_TIER(29.9, 71.69, 55, 1f, 120),
        AUTON_HIGH_TIER(22.07, 110, 62.1226, 1f, 180),

        SHARED_SHIPPING_HUB(75, 130, 20, 1.5f, 130),

        AUTON_FFUTSE_UP(0, 0, 0, 0, 0, 0),
        AUTON_FFUTSE_HOME(0, 0, -90, 0, 0, 0),
        STOW_FFUTSE(0, 0, -90, 0, 0, 0),
        RELEASE_FFUTSE(0, 0, -90, 0, 0, 0),

        AUTON_FFUTSE_LEFT(75, 130, 20, -30, 1.5f, 130),
        AUTON_FFUTSE_MIDDLE(75, 130, 20, 0, 1.5f, 130),
        AUTON_FFUTSE_RIGHT(75, 130, 20, 30, 1.5f, 130),

        CAP(30, 140, 0, 0, 1, 170);

        public double shoulderPos, elbowPos, wristPos;
        public double turretAngle;
        public float toHomeTime;
        public double dumpPos;
        public boolean turret;

        Articulation(double shoulderPos, double elbowPos, double wristPos, double turretAngle, float toHomeTime, double dumpPos) {
            this.shoulderPos = shoulderPos;
            this.elbowPos = elbowPos;
            this.wristPos = wristPos;
            this.turretAngle = turretAngle;
            this.toHomeTime = toHomeTime;
            this.dumpPos = dumpPos;
            turret = true;
        }

        Articulation(double shoulderPos, double elbowPos, double wristPos, float toHomeTime, double dumpPos) {
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
    private final StateMachine main = getStateMachine(new Stage())
            .addSingleState(() -> {
                dumping = false;
            })
            .addTimedState(() -> currentToHomeTime, () -> setTargetPositions(Articulation.HOME), () -> {
            })
            .addTimedState(() -> articulation.toHomeTime / 4, () -> setIntermediatePositions(articulation), () -> {})
            .addTimedState(() -> articulation.toHomeTime / 2 , () -> setTargetPositions(articulation),
                    () -> {
                        currentToHomeTime = articulation.toHomeTime;
                        if (articulation.dumpPos != 0)
                            currentDumpPos = articulation.dumpPos;
                    })

            .build();
    private final StateMachine mainNoHome = getStateMachine(new Stage())
            .addSingleState(() -> {
                dumping = false;
            })
            .addTimedState(() -> articulation.toHomeTime, () -> {
                setTargetPositions(articulation);
                currentToHomeTime = articulation.toHomeTime;
                if (articulation.dumpPos != 0)
                    currentDumpPos = articulation.dumpPos;
            }, () -> {})
            .build();

    private final StateMachine init = getStateMachine(new Stage())
            .addTimedState(2f, () -> setTargetPositions(Articulation.INIT), () -> {
            })
            .build();

    public boolean articulate(Articulation articulation) {
        if (articulation.equals(Articulation.MANUAL))
            return true;
        else if (articulation.equals(Articulation.INIT)) {
            this.articulation = articulation;
            if (init.execute()) {
                this.articulation = Articulation.MANUAL;
                return true;
            }
        } else {
            this.articulation = articulation;
            if (toHomeEnabled ? main.execute() : mainNoHome.execute()) {
                this.articulation = Articulation.MANUAL;
                return true;
            }
        }
        return false;
    }

    @Override
    public void update(Canvas fieldOverlay) {

        bucketDistance = bucketDistanceSensor.getDistance(DistanceUnit.INCH);

        articulate(articulation);

        if (shoulderTargetAngle > 180)
            shoulderTargetAngle -= 360;
        if (elbowTargetAngle > 180)
            elbowTargetAngle -= 360;
        if (wristTargetAngle > 180)
            wristTargetAngle -= 360;

        shoulderServo.setPosition(servoNormalize(shoulderServoValue(shoulderTargetAngle)));
        elbowServo.setPosition(servoNormalize(elbowServoValue(elbowTargetAngle)));
        wristServo.setPosition(servoNormalize(wristServoValue(wristTargetAngle)));

        if (articulation != Articulation.MANUAL)
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

        if (debug) {
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

    private void setIntermediatePositions(Articulation articulation) {
        double shoulderAngle = (articulation.shoulderPos - Articulation.HOME.shoulderPos) / 2.0;
        double elbowAngle = (articulation.elbowPos - Articulation.HOME.elbowPos) / 2.0;

        setShoulderTargetAngle(shoulderAngle);
        setElbowTargetAngle(elbowAngle);
        setWristTargetAngle(articulation.wristPos);

        if (articulation.turret)
            turret.setTargetHeading(articulation.turretAngle);
    }

    private void setTargetPositions(Articulation articulation) {
        setShoulderTargetAngle(articulation.shoulderPos);
        setElbowTargetAngle(articulation.elbowPos);
        setWristTargetAngle(articulation.wristPos);

        if (articulation.turret)
            turret.setTargetHeading(articulation.turretAngle);
    }

    // ----------------------------------------------------------------------------------------------
    // Getters And Setters
    // ----------------------------------------------------------------------------------------------

    // take the supplied relative-to-home target value in degrees
    // and convert to servo setting
    private double shoulderServoValue(double targetPos) {
        double newPos = Range.clip(targetPos, SHOULDER_DEG_MIN, SHOULDER_DEG_MAX);
        newPos = newPos * SHOULDER_PWM_PER_DEGREE + SHOULDER_HOME_PWM;
        return newPos;
    }

    private double elbowServoValue(double targetPos) {
        double newPos = Range.clip(targetPos, ELBOW_DEG_MIN, ELBOW_DEG_MAX);
        newPos = newPos * ELBOW_PWM_PER_DEGREE + ELBOW_HOME_PWM;
        return newPos;
    }

    private double wristServoValue(double targetPos) {
        double newPos = Range.clip(targetPos, WRIST_DEG_MIN, WRIST_DEG_MAX);
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

    public Articulation getArticulation() {
        return articulation;
    }

    public boolean isDumping() {
        return dumping;
    }

    public double getBucketDistance() {
        return bucketDistance;
    }

    public void setToHomeEnabled(boolean toHomeEnabled) {
        this.toHomeEnabled = toHomeEnabled;
    }
}
