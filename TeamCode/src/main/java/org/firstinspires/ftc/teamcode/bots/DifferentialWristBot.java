package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DifferentialWristBot extends PivotBotTest{

    private Servo leftDifferentialWristServo;
    private Servo rightDifferentialWristServo;

    public final double MIN_ANGLE = -90.0;
    public final double MAX_ANGLE = 90.0;
    public final double MIN_SERVO_POS = 0.0;
    public final double MAX_SERVO_POS = 1.0;
    public final double GEAR_RATIO = 2.0;

    private double currentPitch = 0.0;
    private double currentRoll = 0.0;

    public DifferentialWristBot(LinearOpMode opMode) {
        super(opMode);
    }

    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        leftDifferentialWristServo = ahwMap.get(Servo.class, "leftDifferentialWristServo");
        rightDifferentialWristServo = ahwMap.get(Servo.class, "rightDifferentialWristServo");

//         Initialize position
        pitchTo(0);
        rollTo(0);
    }

    protected void onTick() {
        super.onTick();
    }

    public double pitch(double delta) {
        currentPitch = clampAngle(currentPitch + delta);
        updateServos();
        return currentPitch;
    }

    public double pitchTo(double targetAngle) {
        currentPitch = clampAngle(targetAngle);
        updateServos();
        return currentPitch;
    }

    public double roll(double delta) {
        currentRoll = clampAngle(currentRoll + delta);
        updateServos();
        return currentRoll;
    }

    public double rollTo(double targetAngle) {
        currentRoll = clampAngle(targetAngle);
        updateServos();
        return currentRoll;
    }

    /**
     * Updates the servos based on the stored pitch and roll values.
     * Ensures proper movement behavior for both pitch and roll.
     */
    private void updateServos() {
        double leftAngle = currentRoll + currentPitch;  // Roll moves both the same, Pitch moves oppositely
        double rightAngle = currentRoll - currentPitch;

        double leftPos = clamp(angleToServo(leftAngle));
        double rightPos = clamp(angleToServo(rightAngle));

        // Set both servos simultaneously
        leftDifferentialWristServo.setPosition(leftPos);
        rightDifferentialWristServo.setPosition(rightPos);

        // Update telemetry (if needed, but ensure it's non-blocking)
        telemetry.addData("Left Servo Target", leftPos);
        telemetry.addData("Right Servo Target", rightPos);

    }

    private double angleToServo(double angle) {
        return (angle - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE);
    }

    private double servoToAngle(double pos) {
        return pos * (MAX_ANGLE - MIN_ANGLE) + MIN_ANGLE;
    }

    private double clamp(double value) {
        return Math.max(MIN_SERVO_POS, Math.min(MAX_SERVO_POS, value));
    }

    private double clampAngle(double angle) {
        return Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angle));
    }

    public void setLeftDifferentialWristServo(double pos) {
        leftDifferentialWristServo.setPosition(pos);
    }
    public void setRightDifferentialWristServo(double pos) {
        rightDifferentialWristServo.setPosition(pos);
    }
}
