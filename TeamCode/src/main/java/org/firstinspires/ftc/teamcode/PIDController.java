package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import android.util.Log;

public class PIDController {

    String name;
    double integral;
    boolean mecanum;
    double KP, KI, KD;
    double lastTime, lastError;
    double TICKS_IN_THIRTY_INCHES = convertInchesToTicks(30);

    //todo: make inches to ticks and error tolerance fields

    public PIDController (String name, double KP, double KI, double KD, boolean mecanum) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.name = name;

        this.mecanum = mecanum;

        this.lastTime = SystemClock.elapsedRealtimeNanos() * 0.000001;
        this.lastError = 0;
    }

    public double calculatePID (double currentPos, double targetPos) {
        double currentTime = SystemClock.elapsedRealtimeNanos() * 0.000001;
        double deltaTime = currentTime - lastTime;
        double currentError = targetPos - currentPos;
        double deltaError = currentError - lastError;
        double averageError = (currentError + lastError) / 2;
        double rampUpTicks = convertInchesToTicks(5);
        double power;
        double ticksInThirty = convertInchesToTicks(30);

        Log.d("new pid", "calculatePID: ticks in thirty inches " + ticksInThirty);
        Log.d("new pid", "calculatePID: controller name is " + name);

        if (Math.abs(currentPos) < rampUpTicks) {

            if (targetPos > currentPos) {
                power = 0.3 + (0.7 * (currentPos / rampUpTicks));
            } else {
                power = -0.3 + (0.7 * (currentPos / rampUpTicks));
            }

            Log.d("new pid", "calculatePID: ramping up, current position is " + currentPos);
            Log.d("new pid", "calculatePID: ramping up, current error is " + currentError);
            Log.d("new pid", "calculatePID: ramping up, power is " + power);

            // set up for next loop
            lastError = currentError;
            lastTime = currentTime;

        } else {
            if (Math.abs(currentError) < TICKS_IN_THIRTY_INCHES) {
                integral += averageError * deltaTime;
            } else {
                // = 0;
            }

            double derivative = deltaError / deltaTime;

            power = currentError * KP + integral * KI + derivative * KD;


            // set up for next loop
            lastError = currentError;
            lastTime = currentTime;

            Log.d("new pid", "calculatePID: current error is " + currentError);
            Log.d("new pid", "calculatePID: p component is " + (currentError * KP));
            Log.d("new pid", "calculatePID: integral is " + (integral * KI));
            Log.d("new pid", "calculatePID: derivative is " + (derivative * KD));
            Log.d("new pid", "calculatePID: power is " + power);
        }

        return power;
    }

    public double convertInchesToTicks (double inches) {
        if (!mecanum) {
            final double wheelDiaMm = 96;
            final double wheelCircIn = wheelDiaMm * Math.PI / 25.4;
            final double IN_TO_TICK = 537 / wheelCircIn;

            if (inches >= 0) {
                return inches * IN_TO_TICK + 90;
            } else {
                return inches * IN_TO_TICK - 90;
            }
        } else {
            return inches * 51;
        }
    }
}
