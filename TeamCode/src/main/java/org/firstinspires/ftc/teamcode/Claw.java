package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private DcMotor slideMotor;
    private Servo clawServo;
    private Servo wristServo;
    private Servo elbowServo;
    private TouchSensor limitSwitch;
    
    // Servo positions
    private static final double CLAW_OPEN = 0.5;
    private static final double CLAW_CLOSED = 0.9;
    private static final double WRIST_UP = 0.25;
    private static final double WRIST_DOWN = 0.55;
    private static final double ELBOW_UP = 0.65;      // Fully raised position
    private static final double ELBOW_FORWARD = 0.3;  // Horizontal position
    private static final double ELBOW_DOWN = 0.0;     // Fully lowered position
    
    // Slide positions (in encoder ticks)
    private static final int SLIDE_GROUND = 0;
    private static final int SLIDE_LOW = 800;
    private static final int SLIDE_MEDIUM = 1600;
    private static final int SLIDE_HIGH = 3000;
    
    // Slide movement parameters
    private static final double SLIDE_POWER = 0.5;
    private static final double HOLDING_POWER = 0.1;  // Power to hold against gravity
    private static final int POSITION_TOLERANCE = 10;
    
    public Claw(HardwareMap hardwareMap) {
        // Initialize limit switch first to fail fast if there's an issue
        try {
            limitSwitch = hardwareMap.get(TouchSensor.class, "limit_switch");
            if (limitSwitch == null) {
                throw new RuntimeException("Limit switch was found in hardware map but returned null");
            }
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize limit switch. Check that:\n" +
                "1. Device name is exactly 'limit_switch'\n" +
                "2. Device is configured as REV Touch Sensor\n" +
                "3. Wire connections are secure\n" +
                "Error: " + e.getMessage());
        }
        
        // Initialize slide motor
        try {
            slideMotor = hardwareMap.get(DcMotor.class, "vertical_slide");
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setDirection(DcMotor.Direction.REVERSE);
            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize slide motor: " + e.getMessage());
        }
        
        // Initialize servos
        try {
            clawServo = hardwareMap.get(Servo.class, "claw_servo");
            wristServo = hardwareMap.get(Servo.class, "wrist_servo");
            elbowServo = hardwareMap.get(Servo.class, "elbow_servo");
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize servos: " + e.getMessage());
        }
        
        // Initialize to safe starting position
        closeClaw();
        wristUp();
        elbowUp();  // Start with elbow in raised position
    }
    
    // Claw control methods
    public void openClaw() {
        clawServo.setPosition(CLAW_OPEN);
    }
    
    public void closeClaw() {
        clawServo.setPosition(CLAW_CLOSED);
    }
    
    // Wrist control methods
    public void wristUp() {
        wristServo.setPosition(WRIST_UP);
    }
    
    public void wristDown() {
        wristServo.setPosition(WRIST_DOWN);
    }
    
    // Elbow control methods
    public void elbowUp() {
        elbowServo.setPosition(ELBOW_UP);
    }
    
    public void elbowForward() {
        elbowServo.setPosition(ELBOW_FORWARD);
    }
    
    public void elbowDown() {
        elbowServo.setPosition(ELBOW_DOWN);
    }
    
    // Slide control methods
    public void moveToPosition(int targetPosition) {
        slideMotor.setTargetPosition(targetPosition);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(SLIDE_POWER);
    }
    
    public void moveToGround() {
        // Switch to manual control mode
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // If limit switch is pressed, stop and reset
        if (isLimitSwitchPressed()) {
            slideMotor.setPower(0);
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideMotor.setPower(HOLDING_POWER);
        } else {
            // Move down slowly until limit switch is hit
            slideMotor.setPower(-SLIDE_POWER * 0.5);  // Negative power to move down
        }
    }
    
    public void moveToLow() {
        moveToPosition(SLIDE_LOW);
    }
    
    public void moveToMedium() {
        moveToPosition(SLIDE_MEDIUM);
    }
    
    public void moveToHigh() {
        moveToPosition(SLIDE_HIGH);
    }
    
    // Check if slide is at target position
    public boolean isAtTargetPosition() {
        return Math.abs(slideMotor.getCurrentPosition() - slideMotor.getTargetPosition()) < POSITION_TOLERANCE;
    }
    
    // Check if limit switch is pressed
    public boolean isLimitSwitchPressed() {
        return limitSwitch.isPressed();
    }
    
    // Method to check and handle limit switch during ground movement
    public void checkLimitSwitch() {
        if (isLimitSwitchPressed()) {
            // Stop the motor
            slideMotor.setPower(0);
            // Reset encoder position since we're at the bottom
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Apply holding power
            slideMotor.setPower(HOLDING_POWER);
        }
    }
    
    // Emergency stop for the slide
    public void stopSlide() {
        slideMotor.setPower(HOLDING_POWER);  // Use holding power instead of full stop
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    // Set slide power with automatic holding power when near zero
    public void setSlidePower(double power) {
        if (Math.abs(power) < 0.01) {
            slideMotor.setPower(HOLDING_POWER);
        } else {
            slideMotor.setPower(power);
        }
    }
    
    // Get current slide position
    public int getCurrentPosition() {
        return slideMotor.getCurrentPosition();
    }
    
    // Getter methods for servo positions
    public double getClawPosition() {
        return clawServo.getPosition();
    }
    
    public double getWristPosition() {
        return wristServo.getPosition();
    }
    
    public double getElbowPosition() {
        return elbowServo.getPosition();
    }
    
    // Setter methods for manual control
    public void setClawPosition(double position) {
        clawServo.setPosition(position);
    }
    
    public void setWristPosition(double position) {
        wristServo.setPosition(position);
    }
    
    public void setElbowPosition(double position) {
        elbowServo.setPosition(position);
    }
} 