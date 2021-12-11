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
    int gripperOpenTransfer = 1200;
    int gripperUp = 900;
    int gripperDown = 1650;
    boolean gripperIsUp = true;
    double transferTime = 1.0;
    boolean gripperOpen = false;
    private static final String TELEMETRY_NAME = "Gripper";

    public Gripper(HardwareMap hardwareMap){
        gripperServo = hardwareMap.get(Servo.class, "gripperServo");
        gripperPitchServo = hardwareMap.get(Servo.class, "gripperPitchServo");

        gripperServo.setPosition(servoNormalize(gripperClosed));
        gripperPitchServo.setPosition(servoNormalize(gripperUp));
    }

    public void actuateGripper(boolean open){
            if(open) {
                if(gripperIsUp) {
                    if(safeToTransfer) {
                        gripperServo.setPosition(servoNormalize(gripperOpenTransfer));
                        gripperOpen = true;
                    }
                }
                else {
                    gripperServo.setPosition(servoNormalize(gripperOpenIntake));
                    gripperOpen = true;
                }
            }
            else{
                gripperServo.setPosition(servoNormalize(gripperClosed));
                gripperOpen = false;
            }
    }

    public void pitchGripper(boolean up){
        if(up) {
            gripperPitchServo.setPosition(servoNormalize(gripperUp));
            calculateSTFT = true;
            gripperIsUp = true;
        }
        else{
            gripperPitchServo.setPosition(servoNormalize(gripperDown));
            gripperIsUp = false;
        }
    }

    boolean calculateSTFT = false;
    boolean safeToTransfer = true;
    boolean timerInitialized;
    double safeToTransferTimer = 0;
    private boolean safeToTransfer(double seconds){
        if(timerInitialized){
            safeToTransferTimer = System.nanoTime();
            safeToTransfer = false;
        }

        if(System.nanoTime() - safeToTransferTimer > (seconds * 1E9)){
            timerInitialized = false;
            safeToTransfer = true;
            return true;
        }

        return false;
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
        if(gripperIsUp){
            actuateGripper(false);
        }
        else{
            actuateGripper(true);
        }
    }

    @Override
    public void update() {
        safeToTransfer(transferTime); //update the timer
    }

    @Override
    public void stop() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new HashMap<String, Object>();

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return TELEMETRY_NAME;
    }
}
