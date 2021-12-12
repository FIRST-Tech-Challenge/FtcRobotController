package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.Map;
import static org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.UtilMethods.servoNormalize;

public class Gripper implements Subsystem{
    private Servo gripperPitchServo = null;
    private Servo gripperServo = null;
    int gripperClosed =900;
    int gripperOpenIntake = 1200;
    int gripperUp = 2100;
    int gripperDown = 1240;
    boolean gripperIsUp = true;
    boolean gripperOpen = true;

    int gripperTargetPos = 0;

    private static final String TELEMETRY_NAME = "Gripper";

    public Gripper(HardwareMap hardwareMap){
        gripperServo = hardwareMap.get(Servo.class, "gripperServo");
        gripperPitchServo = hardwareMap.get(Servo.class, "gripperPitchServo");

        gripperServo.setPosition(servoNormalize(gripperOpenIntake));
        gripperPitchServo.setPosition(servoNormalize(gripperUp));
    }

    public void actuateGripper(boolean open){
        if(open) {
            gripperTargetPos = gripperOpenIntake;
            gripperOpen = true;
        }
        else{
            gripperTargetPos = gripperClosed;
            gripperOpen = false;
        }
    }

    public void pitchGripper(boolean up){
        if(up) {
            gripperPitchServo.setPosition(servoNormalize(gripperUp));
            gripperIsUp = true;
        }
        else{
            gripperPitchServo.setPosition(servoNormalize(gripperDown));
            gripperIsUp = false;
        }
    }


    public void togglePitch(){
        if(gripperIsUp){
            pitchGripper(false);
        }
        else{
            pitchGripper(true);
        }
    }

    public void toggleGripper(){
        if(gripperOpen){
            actuateGripper(false);
        }
        else{
            actuateGripper(true);
        }
    }

    @Override
    public void update() {
        gripperServo.setPosition(servoNormalize(gripperTargetPos));
    }

    @Override
    public void stop() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new HashMap<String, Object>();

        if(debug) {
            telemetryMap.put("GripperTargetPos", gripperTargetPos);

        }

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return TELEMETRY_NAME;
    }
}
