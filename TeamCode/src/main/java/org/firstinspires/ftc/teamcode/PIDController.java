package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double p;
    private double i;
    private double d;
    private double integralSummation;
    private double lastError;
    private ElapsedTime timer;
    private double previousReference = 0;

    public void resetPID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public void init(double Xp, double Xi, double Xd) {
        p = Xp;
        i = Xi;
        d = Xd;
        timer = new ElapsedTime();
    }
    public double getOutput(double state, double reference) {
        double error = reference - state;
        if (reference != previousReference) {
            integralSummation = 0;
            lastError = error;
        }
        integralSummation += error;
        double derivative = ((error - lastError) / timer.seconds());
        lastError = error;
        previousReference = reference;
        return (error * p) + (derivative * d) + (integralSummation * i);
    }
}