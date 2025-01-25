package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel {
    private Servo flywheel;
    private boolean isRunning = false;
    private boolean isForward = true;
    
    // Constants for servo positions
    private static final double STOP_POSITION = 0.5;  // Center position (no rotation)
    private static final double MAX_FORWARD = 1.0;    // Full speed forward
    private static final double MAX_REVERSE = 0.0;    // Full speed reverse
    
    public Flywheel(HardwareMap hardwareMap, String servoName) {
        flywheel = hardwareMap.get(Servo.class, servoName);
        stop(); // Initialize in stopped position
    }
    
    // Start the flywheel in specified direction
    public void start(boolean forward) {
        isRunning = true;
        isForward = forward;
        flywheel.setPosition(forward ? MAX_FORWARD : MAX_REVERSE);
    }
    
    // Stop the flywheel
    public void stop() {
        isRunning = false;
        flywheel.setPosition(STOP_POSITION);
    }
    
    // Toggle the flywheel on/off (maintaining direction)
    public void toggle() {
        if (isRunning) {
            stop();
        } else {
            start(isForward);
        }
    }
    
    // Get current state
    public boolean isRunning() {
        return isRunning;
    }
    
    public boolean isForward() {
        return isForward;
    }
} 