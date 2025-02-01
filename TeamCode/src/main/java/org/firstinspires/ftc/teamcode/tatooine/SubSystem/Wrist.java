package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tatooine.utils.mathUtil.MathUtil;
import org.firstinspires.ftc.teamcode.tatooine.utils.DebugUtils;  // Import DebugUtils for logging

public class Wrist {

    // Define the subsystem name as a constant
    private static final String SUBSYSTEM_NAME = "Wrist";
    private boolean IS_DEBUG_MODE = false; // Instance variable for debug mode

    private Servo wristAngle = null;
    private Servo angleRight = null;
    private Servo angleLeft = null;
    private OpMode opMode = null;
    private AnalogInput angleSensor = null;

    private final double FRONT = 0;
    private final double STRAIGHT = 0;
    private final double INTAKE_FLAT = 0;

    private final double INTAKE_UP = 0;
    private final double SCORE_SAMPLE = 0;
    private final double HOME = 0;
    private final double BACK = 1;

    private final double WRIST_ANGLE_TOLERANCE = 0;

    // Constructor with OpMode and debug mode flag
    public Wrist(OpMode opMode, boolean isDebugMode) {
        this.opMode = opMode;
        this.IS_DEBUG_MODE = isDebugMode; // Set debug mode based on the constructor parameter

        wristAngle = opMode.hardwareMap.get(Servo.class, "WA");
        angleRight = opMode.hardwareMap.get(Servo.class, "WAR");
        angleLeft = opMode.hardwareMap.get(Servo.class, "WAL");
        angleSensor = opMode.hardwareMap.get(AnalogInput.class, "WAS");

        init();
    }

    // Initialization of servos and directions
    public void init() {
        wristAngle.setDirection(Servo.Direction.FORWARD);
        angleRight.setDirection(Servo.Direction.FORWARD);
        angleLeft.setDirection(Servo.Direction.FORWARD);

        // Debugging initialization
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Servo directions set to FORWARD");
    }

    // Get current wrist angle from the sensor
    public double getWristAngle() {
        double angle = -MathUtil.normalizeAngleTo180(MathUtil.voltageToDegrees(angleSensor.getVoltage()));

        // Debugging wrist angle
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Current wrist angle", angle);

        return angle;
    }

    // Set wrist angle position based on a normalized position (0 to 1)
    public void setPositionWristAngle(double position) {
        wristAngle.setPosition(position);

        // Debugging wrist angle position
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Setting wrist angle to position", position);
    }

    // Set wrist angle based on a degree value (0 to 180 degrees)
    public void setAngleWristAngle(double angle) {
        double position = angle / 180;
        setPositionWristAngle(position);

        // Debugging wrist angle in degrees
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Setting wrist angle to: ", angle + " degrees");
    }

    // Set the wrist position for both servos (right and left)
    public void setPositionWrist(double position) {
        angleRight.setPosition(position);
        angleLeft.setPosition(position);

        // Debugging wrist position
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Setting wrist position to", position);
    }

    // Move wrist to the BACK position
    public void back() {
        setPositionWrist(BACK);
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Moving wrist to BACK position");
    }

    // Move wrist to the FRONT position
    public void front() {
        setPositionWrist(FRONT);
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Moving wrist to FRONT position");
    }

    // Move wrist to the STRAIGHT position
    public void straight() {
        setPositionWrist(STRAIGHT);
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Moving wrist to STRAIGHT position");
    }

    // Move wrist to the HOME position
    public void home() {
        setPositionWrist(HOME);
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Moving wrist to HOME position");
    }

    // Move wrist to the INTAKE position
    public void intakeFlat() {
        setPositionWrist(INTAKE_FLAT);
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Moving wrist to INTAKE_FLAT position");
    }

    public void intakeUp(){
        setPositionWrist(INTAKE_UP);
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Moving wrist to INTAKE_UP position");
    }

    // Move wrist to the SCORE_SAMPLE position
    public void scoreSample() {
        setPositionWrist(SCORE_SAMPLE);
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Moving wrist to SCORE_SAMPLE position");
    }

    // Return an action to move the wrist to the specified angle
    public Action moveWristAngle(double goal) {
        MoveWristAngle moveWristAngle = new MoveWristAngle(goal);
        return moveWristAngle;
    }

    // Inner class to define an action for moving the wrist angle
    public class MoveWristAngle implements Action {
        private double goal;

        public MoveWristAngle(double goal) {
            this.goal = goal;
        }

        // Run method for executing the wrist angle move
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setPositionWristAngle(goal);

            // Debugging wrist angle movement
            DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Moving wrist to goal", goal);

            // Return false to keep the action running until the goal is reached
            return !MathUtil.inTolerance(goal, getWristAngle(), WRIST_ANGLE_TOLERANCE);
        }
    }
}
