package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel {
    private CRServo flywheel;
    
    // Power values for continuous rotation
    private static final double STOP_POWER = 0.0;    // No power = stop
    private static final double MAX_POWER = 1.0;     // Full power
    
    private boolean isRunning = false;
    private boolean isForward = true;
    
    public Flywheel(HardwareMap hardwareMap, String servoName) {
        flywheel = hardwareMap.get(CRServo.class, servoName);
        stop(); // Initialize to stopped position
    }
    
    public void start(boolean forward) {
        isRunning = true;
        isForward = forward;
        flywheel.setPower(forward ? -MAX_POWER : MAX_POWER);
    }
    
    public void stop() {
        isRunning = false;
        flywheel.setPower(STOP_POWER);
    }
    
    public void toggle() {
        if (isRunning) {
            stop();
        } else {
            start(isForward);
        }
    }
    
    public boolean isRunning() {
        return isRunning;
    }
    
    public boolean isForward() {
        return isForward;
    }
} 