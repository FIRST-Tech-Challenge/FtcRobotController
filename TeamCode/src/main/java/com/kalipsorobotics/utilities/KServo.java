package com.kalipsorobotics.utilities;

import android.util.Log;

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

    private double targetPosition;
    private final ElapsedTime time = new ElapsedTime();
    private double currentPosition;
    private double currentTime;
    private double prevPosition;
    private double prevTime;
    private double startTime;
    private double estimatedFinishTime;

    private int counter = 0;

    public double estimateTimeMs(double currentPosition, double targetPosition) {
        double deltaPosition = Math.abs(targetPosition - currentPosition);
        double time = deltaPosition * rangeDegrees * (1000 / servoSpeed);
        //0.2 * 255deg * (0.105sec / 60deg) = 0.0892sec
        return time;
        //0.5 * 300deg * 0.25sec/60deg
    }

    public void setTargetPosition(double position) {
        targetPosition = position;
        if (counter == 0) {
            estimatedFinishTime = estimateTimeMs(servo.getPosition(), position) + 10;
            startTime = System.currentTimeMillis();
        }
        servo.setPosition(position);
        counter = counter + 1;
//        time.reset();
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

    public double getTime() {
        return System.currentTimeMillis();
    }

    public boolean isDone() {
        double deltaTime = (getTime() - startTime);
        Log.d("isDone", "time" + deltaTime);
        if (deltaTime > estimatedFinishTime) {
            Log.d("isDone", "done time " + deltaTime + ", port# " + getPortNumber());
            counter = 0;
            return true;
        }
        return false;
    }


    public void setPosition(double position) {
        servo.setPosition(position);
    }


    public double getTargetPosition() {
        return targetPosition;
    }

    public int getPortNumber() {
        return servo.getPortNumber();
    }

    public void setDirection(Servo.Direction direction) {
        servo.setDirection(direction);
    }

    public double getPosition() {
        return servo.getPosition();
    }

    public Servo getServo() {
        return servo;
    }
}