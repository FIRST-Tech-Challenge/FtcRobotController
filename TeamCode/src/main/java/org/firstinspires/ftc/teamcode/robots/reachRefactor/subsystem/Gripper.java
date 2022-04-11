package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.CRServoSim;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.DistanceSensorSim;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.ServoSim;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Utils.*;

@Config
public class Gripper implements Subsystem {
    public static int CLOSED = 1300;
    public static int DUCKCLOSED = 1200;
    public static int RELEASE = 1700;
    public static int OPEN = 1500;
    public static int DUCKOPEN = 2000;
    public static int PITCH_TRANSFER = 2100;
    public static int PITCH_DOWN = 800;
    public static int PITCH_VERTICAL = 1800;
    public static int FREIGHT_TRIGGER = 40; //mm distance to trigger Lift articulation
    public static int MAX_FREIGHT_DISTANCE = 80;
    public static double INTAKE_POWER = 0.3;
    public static double P = 0.975;

    private final Servo pitchServo, servo;
    private final CRServo intakeServo, intakeServoToo;
    private final DistanceSensor freightSensor;

    // State
    boolean up = true;
    boolean open = true;

    private int targetPos, pitchTargetPos;
    private double intakePower;

    private double freightDistance;

    private Articulation articulation;
    private final Map<Gripper.Articulation, StateMachine> articulationMap;

    public Gripper(HardwareMap hardwareMap, boolean simulated){
        if(simulated) {
            servo = new ServoSim();
            pitchServo = new ServoSim();
            intakeServo = new CRServoSim();
            intakeServoToo = new CRServoSim();
            freightSensor = new DistanceSensorSim(100);
        } else {
            servo = hardwareMap.get(Servo.class, "gripperServo");
            pitchServo = hardwareMap.get(Servo.class, "gripperPitchServo");
            freightSensor = hardwareMap.get(RevColorSensorV3.class, "freightSensor");
            intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
            intakeServoToo = hardwareMap.get(CRServo.class, "intakeServoToo");
            intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        articulation = Gripper.Articulation.MANUAL;

        articulationMap = new HashMap<>();
        articulationMap.put(Articulation.SET,set);
        articulationMap.put(Articulation.SETDUCK,setDuck);
        articulationMap.put(Articulation.LIFT, lift);
        articulationMap.put(Articulation.LIFTDUCK, liftDuck);
        articulationMap.put(Articulation.TRANSFER, transfer);
    }

    @Override
    public void update(Canvas fieldOverlay) {
        freightDistance = freightSensor.getDistance(DistanceUnit.MM);

        articulate(articulation);

        if(targetPos == OPEN && pitchTargetPos == PITCH_DOWN) {
            double c = -MAX_FREIGHT_DISTANCE / Math.log(1-P);
            intakePower = Math.max(0, 0.355 * (1 - Math.exp(-(freightDistance - FREIGHT_TRIGGER) / c)));
        }

        servo.setPosition(servoNormalize(targetPos));
        pitchServo.setPosition(servoNormalize(pitchTargetPos));
        intakeServo.setPower(intakePower);
        intakeServoToo.setPower(intakePower);
    }

    public void stop() {
        articulation = Articulation.MANUAL;
        targetPos = 750;
        pitchTargetPos = 750;
    }

    public enum Articulation {
        MANUAL,
        GRIP,
        TRANSFER,
        SET, //Set for Intaking - can be used as an emergency release
        SETDUCK, //Set gripper as duck trap
        LIFT, //Grip and lift to vertical
        LIFTDUCK
    }

    public boolean articulate(Gripper.Articulation articulation) {

        this.articulation = articulation;

            if(articulation.equals(Gripper.Articulation.MANUAL))
                return true;
            else if(articulationMap.get(articulation).execute()) {
                this.articulation = Gripper.Articulation.MANUAL;
                return true;
            }
            return false;
        }

    //Set the gripper for intake - assume this is coming down from the released transfer position
    //Elevation is down and jaws are open wide to just prevent 2 boxes slipping in
    //Do not assume that we want to Set directly out of Transfer - there may be barriers to cross
    private final Stage setStage = new Stage();
    private final StateMachine set = getStateMachine(setStage)
            .addSingleState(() -> setIntakePower(-1.0))
            .addSingleState(()->{setTargetPos(CLOSED);}) //close the gripper so it's less likely to catch something
            .addTimedState(.25f, () -> setPitchTargetPos(PITCH_DOWN), () -> {})
            .addTimedState(.5f, ()->{setTargetPos(OPEN);}, () -> {})
            .addSingleState(() -> setIntakePower(INTAKE_POWER))
            .build();

    //set Duck trap Set the gripper for intake - assume this is coming down from the released transfer position
    //Elevation is down and jaws are open wide to just prevent 2 boxes slipping in
    //Do not assume that we want to Set directly out of Transfer - there may be barriers to cross
    private final Stage setDuckStage = new Stage();
    private final StateMachine setDuck = getStateMachine(setDuckStage)
            .addSingleState(() -> setIntakePower(-1.0))
            .addSingleState(()->{setTargetPos(CLOSED);}) //close the gripper so it's less likely to catch something
            .addTimedState(.25f, () -> setPitchTargetPos(PITCH_DOWN), () -> {})
            .addTimedState(.5f, ()->{setTargetPos(DUCKOPEN);}, () -> {})
            .addSingleState(() -> setIntakePower(0)) //stop the intakes
            .build();

    //Gripper closes and lifts to the Transfer-ready position
    //Gripper remains closed - Transfer is separate
    private final Stage liftStage = new Stage();
    private final StateMachine lift = getStateMachine(liftStage)
            .addSingleState(() -> setIntakePower(0))
            .addTimedState(() -> 1f, () -> setTargetPos(CLOSED), () -> {})//close the gripper
            .addTimedState(() -> .2f, () -> setPitchTargetPos(PITCH_VERTICAL), () -> {})
            .build();

    //Gripper closes and lifts to the Transfer-ready position
    //Gripper remains closed - Transfer is separate
    private final Stage liftDuckStage = new Stage();
    private final StateMachine liftDuck = getStateMachine(liftDuckStage)
            .addSingleState(() -> setIntakePower(1)) //start intake
            .addTimedState(() -> .5f, () -> setTargetPos(CLOSED), () -> {})//close the gripper ultra tight (for ducks)
            .addSingleState(() -> setIntakePower(0)) //start intake
            .addTimedState(() -> .2f, () -> setPitchTargetPos(PITCH_VERTICAL), () -> {})
            .build();


    private final Stage transferStage = new Stage();
    private final StateMachine transfer = getStateMachine(transferStage)
            .addTimedState(() -> .5f, () -> setPitchTargetPos(PITCH_TRANSFER), () -> {})//give freight last-second momentum toward the bucket
            .addSingleState(() -> setIntakePower(1.0))
            .addTimedState(() -> .75f, () -> setTargetPos(RELEASE), () -> {})
            .addSimultaneousStates(
                    () -> { setTargetPos(CLOSED); return true; },
                    () -> { setPitchTargetPos(PITCH_VERTICAL); return true; }
            )
            .addSingleState(() -> setIntakePower(0.0))
            .build();

    //Prepare for intake
    public void set() {
        articulation = Articulation.SET;
    }

    //Prepare for duck intake
    public void setDuck() {
        articulation = Articulation.SETDUCK;
    }

    // grip and lift into Transfer position - this might need timing
    public void lift() {
        articulation = Articulation.LIFT;
    }

    // grip and lift into Transfer position - this might need timing
    public void liftDuck() {
        articulation = Articulation.LIFTDUCK;
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();

        if(debug) {
            telemetryMap.put("Servo Target Pos", targetPos);
            telemetryMap.put("Pitch Servo Target Pos", pitchTargetPos);
            telemetryMap.put("intake power", intakePower);
            telemetryMap.put("Open", open);
            telemetryMap.put("Up", up);
            telemetryMap.put("Freight Distance", freightDistance);
            telemetryMap.put("articulation", articulation);
        }

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "Gripper";
    }

    //----------------------------------------------------------------------------------------------
    // Getters And Setters
    //----------------------------------------------------------------------------------------------

    public int getTargetPos() {
        return targetPos;
    }

    private void setTargetPos(int targetPos) {
        this.targetPos = targetPos;
    }

    public void setTargetPosDiag(int targetPos) {
        this.targetPos = targetPos;
    }

    public int getPitchTargetPos() {
        return pitchTargetPos;
    }

    public double getFreightDistance(){return freightDistance;}

    private void setPitchTargetPos(int pitchTargetPos) {
        this.pitchTargetPos = pitchTargetPos;
    }
    public void setPitchTargetPosDiag(int pitchTargetPos) {
        this.pitchTargetPos = pitchTargetPos;
    }
    public void setIntakePower(double intakePower) { this.intakePower = intakePower; }

    public Articulation getArticulation() {
        return articulation;
    }
}
