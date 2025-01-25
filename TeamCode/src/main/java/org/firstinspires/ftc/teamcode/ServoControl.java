package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ServoControl {
    private Servo servo;
    private double minPosition = 0.0;
    private double maxPosition = 1.0;
    
    public ServoControl(HardwareMap hardwareMap, String servoName) {
        servo = hardwareMap.get(Servo.class, servoName);
    }
    
    // Set custom min/max position limits
    public void setLimits(double min, double max) {
        minPosition = Math.max(0.0, Math.min(1.0, min));
        maxPosition = Math.max(0.0, Math.min(1.0, max));
    }
    
    // Move to a specific position (0.0 to 1.0)
    public void setPosition(double position) {
        // Clamp position between min and max
        double clampedPosition = Math.max(minPosition, Math.min(maxPosition, position));
        servo.setPosition(clampedPosition);
    }
    
    // Move to a specific angle (0 to 180 degrees)
    public void setAngle(double degrees) {
        // Convert degrees to position (0-180 to 0-1)
        double position = degrees / 180.0;
        setPosition(position);
    }
    
    // Get current position
    public double getPosition() {
        return servo.getPosition();
    }
    
    // Get current angle
    public double getAngle() {
        return servo.getPosition() * 180.0;
    }
} 