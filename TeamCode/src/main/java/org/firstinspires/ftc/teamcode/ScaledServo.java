package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ScaledServo {
    private final String logName;
    private final Servo servo;

    public ScaledServo(Servo servo, String name, double servoMin, double servoMax) {
        logName = String.format("Servo %s", name);
        this.servo = servo;
        this.servoMin = servoMin;
        this.servoMax = servoMax;
    }

    private double targetPosition;
    public double getTargetPosition() { return targetPosition; }
    public void setTargetPosition(double targetPosition) {
        this.targetPosition = Math.min(1.0, Math.max(0.0, targetPosition));
        updateServoPosition();
    }

    private void updateServoPosition() {
        double servoRange = servoMax - servoMin;
        double servoPosition = servoMin + (servoRange * targetPosition);

        servo.setPosition(servoPosition);
    }

    public double getPosition() {
        double servoPosition = servo.getPosition();
        double servoRange = servoMax - servoMin;

        return (servoPosition - this.servoMin) / servoRange;
    }

    private double servoMax;
    public double getServoMax() { return servoMax; }
    public void setServoMax(double servoMax) {
        this.servoMax = Math.min(1.0, Math.max(servoMin + 0.01, servoMax));
        updateServoPosition();
    }
    public void adjustServoMax(double delta) {
        setServoMax(servoMax + delta);
    }

    private double servoMin;
    public double getServoMin() { return servoMin; }
    public void setServoMin(double servoMin) {
        this.servoMin = Math.max(0.0, Math.min(servoMax - 0.01, servoMin));
        updateServoPosition();
    }
    public void adjustServoMin(double delta) {
        setServoMin(servoMin + delta);
    }

    public void dumpTelemetry(Telemetry telemetry) {
        telemetry.addData(logName, "%6.4f / %6.4f / %6.4f -> %6.4f",
            servoMin, servo.getPosition(), servoMax, targetPosition);
    }
}
