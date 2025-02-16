package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    private DcMotor intakeMotor;
    private Servo intakeServo;
    private Flywheel flywheel;
    private ElapsedTime timer;
    private boolean isOut = false; // Tracks whether intake is out (false = in, true = out)
    private static final double INTAKE_POWER = 0.5;  // Adjust this value for desired speed
    private static final double SERVO_UP_POSITION = 0.25;    // Servo position for intake up
    private static final double SERVO_MID_POSITION = 0.6;  // Servo position for intake mid
    private static final double SERVO_DOWN_POSITION = 1.0;  // Servo position for intake down (180 degrees)
    private static final double MOTOR_RUN_TIME_MS = 500;  // Time to run motor in milliseconds
    
    public Intake(HardwareMap hardwareMap, String motorName) {
        intakeMotor = hardwareMap.get(DcMotor.class, motorName);
        intakeServo = hardwareMap.get(Servo.class, "intake_wrist");
        flywheel = new Flywheel(hardwareMap, "flywheel");
        timer = new ElapsedTime();
        stop(); // Initialize in stopped position
        raiseIntake(); // Start with intake raised
        isOut = false; // Initialize state as in
    }
    
    // Move the intake forwards
    public void forward() {
        intakeMotor.setPower(-INTAKE_POWER);
    }
    
    // Move the intake backwards
    public void backward() {
        intakeMotor.setPower(INTAKE_POWER);
    }
    
    // Stop the intake
    public void stop() {
        intakeMotor.setPower(0);
    }

    // Lower the intake mechanism
    public void lowerIntake() {
        intakeServo.setPosition(SERVO_DOWN_POSITION);
    }

    public void midIntake() {
        intakeServo.setPosition(SERVO_MID_POSITION);
    }

    // Raise the intake mechanism
    public void raiseIntake() {
        intakeServo.setPosition(SERVO_UP_POSITION);
    }

    // Get current state of intake
    public boolean isIntakeOut() {
        return isOut;
    }

    // Raise the intake mechanism
    public void up(boolean Up) {
        // Stop flywheel
        flywheel.stop();
        
        // Only move servo if not already up
        if (isOut) {
            if (Up) {
                // Raise intake mechanism
                raiseIntake();
            } else {
                // Raise intake mechanism
                midIntake();
            }
            isOut = false;
        }
    }

    // Lower the intake mechanism
    public void down(boolean flywheelForward) {
        // Only move servo if not already down
        if (!isOut) {
            // Lower intake mechanism
            lowerIntake();
            isOut = true;
            
            // Run motor outward
            forward();
            timer.reset();
            while (timer.milliseconds() < MOTOR_RUN_TIME_MS) {
                // Wait for timer
            }
            stop();
        }
        
        // Start flywheel
        flywheel.start(flywheelForward);
    }

    // Retract intake until it stops moving
    public void in(boolean Up) {
        // First raise the intake mechanism
        up(Up);
        
        // Start moving the motor inward
        backward();
        
        // Initialize position tracking
        int lastPosition = intakeMotor.getCurrentPosition();
        timer.reset();
        
        // Keep running until no movement is detected for a short period
        while (timer.milliseconds() < 100) {  // Check for 100ms of no movement
            int currentPosition = intakeMotor.getCurrentPosition();
            if (currentPosition != lastPosition) {
                // Movement detected, reset timer
                timer.reset();
                lastPosition = currentPosition;
            }
        }
        
        // Stop the motor once no movement is detected
        stop();
    }
} 