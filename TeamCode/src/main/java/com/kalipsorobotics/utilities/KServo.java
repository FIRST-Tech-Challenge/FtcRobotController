package com.kalipsorobotics.utilities;

import android.os.SystemClock;
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

    private int counter = 0;

    public void setTargetPosition(double position) {
        servo.setPosition(position);
        targetPosition = position;
        if (counter == 0) {
            startTime = System.currentTimeMillis();
        }
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
        Log.d("isDone", "time" + (getTime() - startTime));
        if ((getTime() - startTime) > 500) {
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

}
