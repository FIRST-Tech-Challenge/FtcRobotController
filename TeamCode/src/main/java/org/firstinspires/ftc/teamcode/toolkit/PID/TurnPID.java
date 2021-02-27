package org.firstinspires.ftc.teamcode.toolkit.PID;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.UpliftRobot;

public class TurnPID {
    double kP, kI, kD, p, i, d, tolerance, pidZone, lastP, maxP, lastTime, output;
    boolean initial;
    public boolean completed;
    UpliftRobot robot;

    public TurnPID(double kP, double kI, double kD, double tolerance, double pidZone, UpliftRobot robot){
        this.robot = robot;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.tolerance = tolerance;
        this.pidZone = pidZone;
        this.p = 0;
        this.i = 0;
        this.d = 0;
        this.lastP = 0;
        this.lastTime = 0;
        this.maxP = 0;
        this.output = 0;
        this.initial = true;
        this.completed = false;
    }

    public double getOutput(double error) {
        p = error;
        if(initial) {
            lastTime = System.currentTimeMillis();
            lastP = p;
            initial = false;
        }
        i += p * (System.currentTimeMillis() - lastTime);
        d = ((p - lastP) / (System.currentTimeMillis() - lastTime));
        lastP = p;
        lastTime = System.currentTimeMillis();

//        if(Math.signum(p) != Math.signum(i)) {
//            i = 0;
//        }

        if(Math.abs(p) < Math.abs(tolerance)){
            output = 0;
            completed = true;
        } else if(Math.abs(p) < Math.abs(pidZone)){
            output = p * kP + i * kI + d * kD;
        } else {
            output = 1;
        }

        return Range.clip(output, -1, 1);
    }

    public void clearPID() {
        p = 0;
        i = 0;
        d = 0;
        lastTime = 0;
        lastP = 0;
        initial = true;
        completed = false;
    }
}
