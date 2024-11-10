package com.kalipsorobotics.PID;

import androidx.annotation.NonNull;

public class PIDState {
    public boolean mecanum;
    public double integral;
    public double derivative;
    public double KP, KI, KD;
    public double lastTime, lastError = 0, lastPos = 0;
    public boolean rampingUp = false;

    public PIDState(double KP, double KI, double KD, double lastTime, boolean mecanum) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;

        this.lastTime = lastTime;
        this.mecanum = mecanum;
    }

    @NonNull
    @Override
    public String toString() {
        return "rampingUp=" + rampingUp + "\n" +
                "error=" + lastError + "\n" +
                "p=" + lastError * KP + "\n" +
                "integral=" + integral * KI + "\n" +
                "derivative=" + derivative * KD + "\n";
    }
}