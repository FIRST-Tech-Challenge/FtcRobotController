package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.Map;
import static org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.UtilMethods.servoNormalize;

@Config
public class Gripper implements Subsystem{
    // Servos
    public Servo pitchServo, servo;

    // State
    boolean up = true;
    boolean open = true;

    private int targetPos = 0;
    private int pitchTargetPos = 0;

    // Constants
    private static final String TELEMETRY_NAME = "Gripper";

    public static int CLOSED = 900;
    public static int OPEN = 1200;
    public static int PITCH_UP = 2100;
    public static int PITCH_DOWN = 1240;
    public static int PITCH_INIT = 1854;

    public Gripper(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "gripperServo");
        pitchServo = hardwareMap.get(Servo.class, "gripperPitchServo");
    }

    public void actuateGripper(boolean open){
        this.open = open;
        targetPos = open ? OPEN : CLOSED;
    }

    public void pitchGripper(boolean up){
        this.up = up;
        pitchTargetPos = up ? PITCH_UP : PITCH_DOWN;
    }


    public void togglePitch(){
        pitchGripper(!up);
    }

    public void toggleGripper(){
        actuateGripper(!open);
    }

    @Override
    public void update() {
        servo.setPosition(servoNormalize(targetPos));
        pitchServo.setPosition(servoNormalize(pitchTargetPos));
    }

    @Override
    public void stop() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new HashMap<String, Object>();

        if(debug) {
            telemetryMap.put("Servo Target Pos", targetPos);
            telemetryMap.put("Pitch Servo Target Pos", pitchTargetPos);
            telemetryMap.put("Open", open);
            telemetryMap.put("Up", up);
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

    public void setTargetPos(int targetPos) {
        this.targetPos = targetPos;
    }

    public int getPitchTargetPos() {
        return pitchTargetPos;
    }

    public void setPitchTargetPos(int pitchTargetPos) {
        this.pitchTargetPos = pitchTargetPos;
    }

}
