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
    private static final double SERVO_UP_POSITION = 0.3;    // Servo position for intake up
    private static final double SERVO_DOWN_POSITION = 1.0;  // Servo position for intake down (180 degrees)
    private static final double MOTOR_RUN_TIME_MS = 1000;  // Time to run motor in milliseconds
    
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

    // Raise the intake mechanism
    public void raiseIntake() {
        intakeServo.setPosition(SERVO_UP_POSITION);
    }

    // Coordinated out sequence: motor forward -> lower intake -> start flywheel
    // @param flywheelForward true for forward direction, false for backward direction
    public void out(boolean flywheelForward) {
        // Only move motor and servo if not already out
        if (!isOut) {
            // Start motor forward
            forward();
            
            // Run motor for 1 second
            timer.reset();
            while (timer.milliseconds() < MOTOR_RUN_TIME_MS) {
                // Wait for motor to run
            }
            stop();
            
            // Lower intake mechanism
            lowerIntake();
            
            isOut = true;
        }
        
        // Always control flywheel regardless of state
        flywheel.start(flywheelForward);
    }

    // Coordinated in sequence: stop flywheel -> raise intake -> motor backward
    public void in() {
        // Always stop flywheel regardless of state
        flywheel.stop();
        
        // Only move motor and servo if not already in
        if (isOut) {
            // Raise intake mechanism
            raiseIntake();
            
            // Run motor backward for 1 second
            backward();
            timer.reset();
            while (timer.milliseconds() < MOTOR_RUN_TIME_MS) {
                // Wait for motor to run
            }
            stop();
            
            isOut = false;
        }
    }

    // Get current state of intake
    public boolean isIntakeOut() {
        return isOut;
    }
} 