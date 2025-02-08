package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DifferentialWristBot extends RollerIntakeBot{

    private Servo leftServo;
    private Servo rightServo;

    private double leftServoPos;
    private double rightServoPos;
    public final double MIN_ANGLE = -90.0;
    public final double MAX_ANGLE = 90.0;
    public final double MIN_SERVO_POS = 0.0;
    public final double MAX_SERVO_POS = 1.0;
    public final double GEAR_RATIO = 2.0;
    public DifferentialWristBot(LinearOpMode opMode)  {
        super(opMode);
    }


    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        leftServo = ahwMap.get(Servo.class, "leftServo");
        rightServo = ahwMap.get(Servo.class, "rightSensor");
        // initialize position
        setServos(0, 0);
    }

    protected void onTick() {
        super.onTick();
    }

    public void pitch(double delta) {
        adjustServos(delta, -delta);
    }

    public void roll(double delta) {
        adjustServos(delta * GEAR_RATIO, delta * GEAR_RATIO);
    }


    private void adjustServos(double leftDelta, double rightDelta) {
        double currentLeftAngle = servoToAngle(leftServo.getPosition());
        double currentRightAngle = servoToAngle(rightServo.getPosition());

        double targetLeftAngle = currentLeftAngle + leftDelta;
        double targetRightAngle = currentRightAngle + rightDelta;

        double targetLeftPos = clamp(angleToServo(targetLeftAngle));
        double targetRightPos = clamp(angleToServo(targetRightAngle));

        // Determine how much movement is actually possible while keeping both servos synchronized
        double maxAllowedChange = Math.min(targetLeftPos - leftServo.getPosition(), targetRightPos - rightServo.getPosition());

        // Apply the adjusted positions to maintain sync
        setServos(leftServo.getPosition() + maxAllowedChange, rightServo.getPosition() + maxAllowedChange);
    }

    private void setServos(double leftPos, double rightPos) {
        leftServo.setPosition(clamp(leftPos));
        rightServo.setPosition(clamp(rightPos));
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
