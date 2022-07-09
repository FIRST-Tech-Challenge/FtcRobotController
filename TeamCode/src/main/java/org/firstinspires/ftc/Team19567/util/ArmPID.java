package org.firstinspires.ftc.Team19567.util;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Attempt at making a PID class with <a href="https://gm0.org/en/latest/docs/software/control-loops.html">gm0</a> as a reference. <br>
 * Was never implemented (or even finished) and is probably incorrect in many ways.
 */

@Deprecated
public class ArmPID {
    private double p;
    private double i;
    private double d;
    private int targetPos;
    private int lastPos = targetPos;
    private int lastError = 0;
    private int error = 0;
    private double output = 0;
    private double sumOfError = 0.0;
    private double lastTime = 0.0;

    private ElapsedTime t = new ElapsedTime();

    public ArmPID(double p, double i, double d, int targetPos) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.targetPos = targetPos;
    }

    public double update(int armPos) {
        error = targetPos-armPos;
        double de = error-lastError;
        double dt = t.seconds()-lastTime;
        sumOfError += (error * dt);
        output = (p * error) + (i * sumOfError) + (d * de/dt);

        return output;
    }
}