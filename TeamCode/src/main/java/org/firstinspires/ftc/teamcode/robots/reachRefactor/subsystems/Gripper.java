package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Map;
import static org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.UtilMethods.servoNormalize;

public class Gripper implements Subsystem{
    private Servo gripperPitchServo = null;
    private Servo gripperServo = null;
    int gripperClosed =900;
    int gripperOpen = 1200;
    int gripperUp = 900;
    int gripperDown = 1650;
    boolean gripperIsUp = true;

    public Gripper(HardwareMap hardwareMap){
        gripperServo = hardwareMap.get(Servo.class, "gripperServo");
        gripperPitchServo = hardwareMap.get(Servo.class, "gripperPitchServo");

        gripperServo.setPosition(servoNormalize(gripperClosed));
        gripperPitchServo.setPosition(servoNormalize(gripperUp));
    }

    public boolean actuateGripper(boolean open){
        if(!gripperIsUp) {
            if(open) {
                gripperServo.setPosition(servoNormalize(gripperOpen));
                return true;
            }
            else{
                gripperServo.setPosition(servoNormalize(gripperOpen));
                return true;
            }
        }
        return false;
    }

//    public boolean pitchGripper(){
//        if()
//    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        return null;
    }

    @Override
    public String getTelemetryName() {
        return null;
    }
}
