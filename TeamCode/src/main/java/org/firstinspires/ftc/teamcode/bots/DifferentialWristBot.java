package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DifferentialWristBot extends FourWheelDriveBot{

    private Servo leftDifferentialWristServo;
    private Servo rightDifferentialWristServo;

    private double leftServoPos;
    private double rightServoPos;
    public final double MIN_ANGLE = -90.0;
    public final double MAX_ANGLE = 90.0;
    public final double MIN_SERVO_POS = 0.0;
    public final double MAX_SERVO_POS = 1.0;
    public final double GEAR_RATIO = 2.0;

    private double currentPitch = 0.0;  // Tracks pitch angle
    private double currentRoll = 0.0;   // Tracks roll angle

    public DifferentialWristBot(LinearOpMode opMode)  {
        super(opMode);
    }


    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        leftDifferentialWristServo = ahwMap.get(Servo.class, "leftServo");
        rightDifferentialWristServo = ahwMap.get(Servo.class, "rightSensor");
        // initialize position
        pitchTo(0);
        rollTo(0);    }

    protected void onTick() {
        super.onTick();
    }
    public double pitch(double delta) {
        adjustServos(delta, -delta);
        return currentPitch;
    }

    public double pitchTo(double targetAngle) {
        double delta = targetAngle - currentPitch;
        return pitch(delta);
    }

    public double roll(double delta) {
        adjustServos(delta * GEAR_RATIO, delta * GEAR_RATIO);
        return currentRoll;
    }

    public double rollTo(double targetAngle) {
        double delta = targetAngle - currentRoll;
        return roll(delta);
    }

    private void adjustServos(double leftDelta, double rightDelta) {
        double targetLeftAngle = currentPitch + leftDelta;
        double targetRightAngle = currentRoll + rightDelta;

        double targetLeftPos = clamp(angleToServo(targetLeftAngle));
        double targetRightPos = clamp(angleToServo(targetRightAngle));

        // Ensure both servos move the same amount if one hits a limit
        double maxAllowedChange = Math.min(targetLeftPos - leftDifferentialWristServo.getPosition(),
                targetRightPos - rightDifferentialWristServo.getPosition());

        double newLeftPos = leftDifferentialWristServo.getPosition() + maxAllowedChange;
        double newRightPos = rightDifferentialWristServo.getPosition() + maxAllowedChange;

        currentPitch = servoToAngle(newLeftPos);
        currentRoll = servoToAngle(newRightPos);

        setServos(newLeftPos, newRightPos);
    }

    private void setServos(double leftPos, double rightPos) {
        leftDifferentialWristServo.setPosition(clamp(leftPos));
        rightDifferentialWristServo.setPosition(clamp(rightPos));
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
}
