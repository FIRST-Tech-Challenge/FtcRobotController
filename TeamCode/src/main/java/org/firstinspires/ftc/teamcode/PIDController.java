package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import android.util.Log;

public class PIDController {

    double integral;
    double KP, KI, KD;
    double lastTime, lastError;

    public PIDController (double KP, double KI, double KD) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;

        this.lastTime = SystemClock.elapsedRealtimeNanos() * 0.000001;
        this.lastError = 0;
    }

    public double calculatePID (double currentPos, double targetPos) {
        double currentTime = SystemClock.elapsedRealtimeNanos() * 0.000001;
        double deltaTime = currentTime - lastTime;
        double currentError = targetPos - currentPos;
        double deltaError = currentError - lastError;
        double averageError = (currentError + lastError) / 2;
        integral += averageError * deltaTime;
        double derivative = deltaError / deltaTime;
        double power = currentError * KP + integral * KI + derivative * KD;

        // set up for next loop
        lastError = currentError;
        lastTime = currentTime;
        Log.d("new pid", "calculatePID: current error is " + currentError);
        Log.d("new pid", "calculatePID: p component is " + (currentError * KP));
        Log.d("new pid", "calculatePID: integral is " + (integral * KI));
        Log.d("new pid", "calculatePID: derivative is " + (derivative * KD));
        Log.d("new pid", "calculatePID: power is " + power);

        return power;
    }

    public double convertInchesToTicks (double inches) {
        final double wheelDiaMm = 96;
        final double PI = 3.14159;
        final double wheelCircIn = wheelDiaMm * PI / 25.4;
        final double IN_TO_TICK = 537 / wheelCircIn;

        return inches * IN_TO_TICK;
    }
}
