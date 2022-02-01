package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.Map;
import static org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.UtilMethods.servoNormalize;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.DistanceSensorSim;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.ServoSim;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.UtilMethods;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

@Config
public class Gripper implements Subsystem{
    private static final String TELEMETRY_NAME = "Gripper";

    public static int CLOSED = 900;
    public static int RELEASE = 1300;
    public static int OPEN = 1350;
    public static int PITCH_TRANSFER = 2250;
    public static int PITCH_DOWN = 830;
    public static int PITCH_INIT = 1854;
    public static int PITCH_VERTICAL = 2100;

    public Servo pitchServo, servo;
    public DistanceSensor freightSensor;

    // State
    boolean up = true;
    boolean open = true;

    private int targetPos = 0;
    private int pitchTargetPos = 0;

    private double freightDistance;

    private Articulation articulation;
    private Map<Gripper.Articulation, StateMachine> articulationMap;

    public Gripper(HardwareMap hardwareMap, boolean simulated){
        if(simulated) {
            servo = new ServoSim();
            pitchServo = new ServoSim();
            freightSensor = new DistanceSensorSim(100);
        } else {
            servo = hardwareMap.get(Servo.class, "gripperServo");
            pitchServo = hardwareMap.get(Servo.class, "gripperPitchServo");
            freightSensor = hardwareMap.get(RevColorSensorV3.class, "freightSensor");
        }

        articulation = Gripper.Articulation.MANUAL;

        articulationMap = new HashMap<>();
        articulationMap.put(Gripper.Articulation.SET,set);
        articulationMap.put(Gripper.Articulation.LIFT, lift);
        articulationMap.put(Articulation.TRANSFER, transfer);
    }

    public void actuateGripper(boolean open){
        this.open = open;
        targetPos = open ? OPEN : CLOSED;
    }

    public void pitchGripper(boolean up){
        this.up = up;
        pitchTargetPos = up ? PITCH_TRANSFER : PITCH_DOWN;
    }


    public void togglePitch(){
        pitchGripper(!up);
    }

    public void toggleGripper(){
        actuateGripper(!open);
    }

    @Override
    public void update(Canvas fieldOverlay) {
        freightDistance = freightSensor.getDistance(DistanceUnit.MM);

        if (pitchServo.getPosition() < 0.5 && freightDistance < 25)
            articulation=Articulation.LIFT;
        articulate(articulation);
        servo.setPosition(servoNormalize(targetPos));
        pitchServo.setPosition(servoNormalize(pitchTargetPos));
    }

    @Override
    public void stop() {

    }

    public enum Articulation {
        MANUAL,
        GRIP,
        TRANSFER,
        SET, //Set for Intaking - can be used as an emergency release
        LIFT //Grip and lift to vertical
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
    private final Stage Set = new Stage();
    private final StateMachine set = UtilMethods.getStateMachine(Set)
            .addSingleState(()->{setTargetPos(CLOSED);}) //close the gripper so it's less likely to catch something
            .addTimedState(() -> .25f, () -> setPitchTargetPos(PITCH_DOWN), () -> {})
            .addSingleState(()->{setTargetPos(OPEN);})
            .build();

    //Gripper closes and lifts to the Transfer-ready position
    //Gripper remains closed - Transfer is separate
    private final Stage Lift = new Stage();
    private final StateMachine lift = UtilMethods.getStateMachine(Lift)
            .addTimedState(() -> .5f, () -> setTargetPos(CLOSED), () -> {})//close the gripper so it's less likely to catch something
            .addTimedState(() -> .5f, () -> setPitchTargetPos(PITCH_VERTICAL), () -> {})
            .build();

    private final Stage Transfer = new Stage();
    private final StateMachine transfer = UtilMethods.getStateMachine(Transfer)
            .addTimedState(() -> .1f, () -> setPitchTargetPos(PITCH_TRANSFER), () -> {})//give freight last-second momentum toward the bucket
            .addTimedState(() -> .75f, () -> setTargetPos(RELEASE), () -> {})
            .addTimedState(() -> 0, () -> setTargetPos(CLOSED), () -> {})
            .addTimedState(() -> 0, () -> setPitchTargetPos(PITCH_VERTICAL), () -> {})
            .build();


    //public void Grip(){}


    public void set() //Prepare for intake
    {articulation=Articulation.SET;}
    public void lift() //grip and lift into Transfer position - this might need timing
    {articulation=Articulation.LIFT;}
    public void transfer() //barely open gripper to release freight into the bucket
    {articulation=Articulation.TRANSFER;}

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new HashMap<>();

        if(debug) {
            telemetryMap.put("Freight Distance", freightSensor.getDistance(DistanceUnit.MM));
            telemetryMap.put("Servo Target Pos", targetPos);
            telemetryMap.put("Pitch Servo Target Pos", pitchTargetPos);
            telemetryMap.put("Open", open);
            telemetryMap.put("Up", up);
            telemetryMap.put("Freight Distance", freightDistance);
        }

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return TELEMETRY_NAME;
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

    private void setPitchTargetPos(int pitchTargetPos) {
        this.pitchTargetPos = pitchTargetPos;
    }
    public void setPitchTargetPosDiag(int pitchTargetPos) {
        this.pitchTargetPos = pitchTargetPos;
    }

}
