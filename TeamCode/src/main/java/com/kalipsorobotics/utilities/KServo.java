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

    private double targetPosition;
    private double lastPosition;
    private final ElapsedTime time = new ElapsedTime();
    private double currentPosition;
    private double currentTime;
    private double prevPosition;
    private double prevTime;
    private double startTime;
    private double estimatedFinishTime;

    public KServo(Servo servo, double servoSpeed, double rangeDegrees, double zeroPosition, boolean flipDirection) {
        this.servo = servo;
        this.servoSpeed = servoSpeed;
        this.rangeDegrees = rangeDegrees;
        this.flipDirection = flipDirection;
        this.zeroPosition = zeroPosition;
        this.lastPosition = servo.getPosition();
    }

    private final int counter = 0;

    public double estimateTimeMs(double currentPosition, double targetPosition) {
        double deltaPosition = Math.abs(targetPosition - currentPosition);
        double time = deltaPosition * rangeDegrees * (1000 / servoSpeed);
        //0.2 * 255deg * (0.105sec / 60deg) = 0.0892sec
        return time;
        //0.5 * 300deg * 0.25sec/60deg
    }

    public void setTargetPosition(double position) {
        lastPosition = servo.getPosition();
        targetPosition = position;
        if(lastPosition != targetPosition) {
            estimatedFinishTime = estimateTimeMs(lastPosition, targetPosition) * 1.25;
            startTime = System.currentTimeMillis();
            servo.setPosition(targetPosition);
        }
//        time.reset();
    }

    public double getCurrentPosition() {
//        double distanceToGo = targetPosition - prevPosition;
//
//        currentTime = time.milliseconds();
//        currentPosition =
//                targetPosition - (distanceToGo - (servoSpeed * (prevTime - currentTime)));
//        prevPosition = currentPosition;
//        prevTime = currentTime;
//        return currentPosition;

        return getServo().getPosition();
    }

    public double getTime() {
        return System.currentTimeMillis();
    }

    public boolean isDone() {
        double deltaTime = (getTime() - startTime);
        Log.d("isDone", "time" + deltaTime);
        if (deltaTime > estimatedFinishTime) {
            Log.d("isDone", "done time " + deltaTime + ", port# " + getPortNumber());
            lastPosition = servo.getPosition();
            return true;
        }
        return false;
    }


    public void setPosition(double position) {
        servo.setPosition(position);
    }

    public boolean isCurrentPosEqualTo(double position) {
        return Math.abs(getServo().getPosition() - position) < 0.00001;
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