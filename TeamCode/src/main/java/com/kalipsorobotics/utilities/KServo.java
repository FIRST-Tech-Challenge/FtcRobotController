package com.kalipsorobotics.utilities;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class KServo {

    private final double servoSpeed; // degrees per sec
    private final Servo servo;
    private final double rangeDegrees;
    private final double zeroPosition;
    private final boolean flipDirection;

    public KServo(Servo servo, double servoSpeed, double rangeDegrees, double zeroPosition, boolean flipDirection) {
        this.servo = servo;
        this.servoSpeed = servoSpeed;
        this.rangeDegrees = rangeDegrees;
        this.flipDirection = flipDirection;
        this.zeroPosition = zeroPosition;
    }

    ElapsedTime time = new ElapsedTime();
    private double targetPosition;
    private double currentPosition;
    private double currentTime;
    private double prevPosition;
    private double prevTime;

    public void setTargetPosition(double position) {
        servo.setPosition(position);
        targetPosition = position;
        time.reset();
    }

    public double getCurrentPosition() {
        double distanceToGo = targetPosition - prevPosition;
        currentTime = time.milliseconds();
        currentPosition =
                targetPosition - (distanceToGo - (servoSpeed * (prevTime - currentTime)));
        prevPosition = currentPosition;
        prevTime = currentTime;
        return currentPosition;
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }


    public double getTargetPosition() {
        return targetPosition;
    }

}
