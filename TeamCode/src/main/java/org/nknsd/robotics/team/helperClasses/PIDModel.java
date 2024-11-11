package org.nknsd.robotics.team.helperClasses;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

public class PIDModel {
    private final double kP;
    private final double kI;
    private final double kD;
    private double errCumulative = 0;
    private double errPrev = 0;
    private long lastCall = 0;


    public PIDModel(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculate(double processVar, double setPoint, ElapsedTime runtime) { // First is the current pos, second is the target pos
        double error = setPoint - processVar;
        long now = runtime.now(TimeUnit.MILLISECONDS) / 10;

        // P
        double p = error * kP;

        // I
        errCumulative += error * (now - lastCall);
        double i = errCumulative * kI;

        // D
        double d = 0;
        if (lastCall != 0) {
            d = (error - errPrev) / (now - lastCall);
            d *= kD;
        }
        errPrev = error;
        lastCall = now;

        return p + i + d;
    }

    public double calculateWithTelemetry(double processVar, double setPoint, ElapsedTime runtime, Telemetry telemetry) { // First is the current pos, second is the target pos
        double error = setPoint - processVar;
        long now = runtime.now(TimeUnit.MILLISECONDS) / 10;

        // P
        double p = error * kP;
        telemetry.addData("P", p);

        // I
        errCumulative += error * (now - lastCall);
        double i = errCumulative * kI;
        telemetry.addData("I", i);

        // D
        double d = 0;
        if (lastCall != 0) {
            d = (error - errPrev) / (now - lastCall);
            d *= kD;
        }
        telemetry.addData("D", d);
        telemetry.addData("Time Between", now - lastCall);
        telemetry.addData("Err Prev", errPrev);
        errPrev = error;
        lastCall = now;

        return p + i + d;
    }

    public void resetError() {
        errCumulative = 0;
        errPrev = 0;
        lastCall = 0; // We can set the lastCall to 0 because if it is 0 then we will skip the initial derivative value.
    }
}
