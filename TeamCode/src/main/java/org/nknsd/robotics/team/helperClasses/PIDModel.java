package org.nknsd.robotics.team.helperClasses;

import com.qualcomm.robotcore.util.ElapsedTime;

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

        // P
        double p = error * kP;

        // I
        errCumulative += error;
        double i = errCumulative * kI;

        // D
        double d = 0;
        if (lastCall != 0) {
            d = (error - errPrev) / (runtime.now(TimeUnit.MILLISECONDS) - lastCall);
            d *= kD;
        }
        errPrev = error;
        lastCall = runtime.now(TimeUnit.MILLISECONDS);

        return p + i + d;
    }

    public void resetError() {
        errCumulative = 0;
        errPrev = 0;
        lastCall = 0; // We can set the lastCall to 0 because if it is 0 then we will skip the initial derivative value.
    }
}
