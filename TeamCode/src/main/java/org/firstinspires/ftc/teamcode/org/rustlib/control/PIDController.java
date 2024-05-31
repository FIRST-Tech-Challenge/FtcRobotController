package org.firstinspires.ftc.teamcode.org.rustlib.control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double kP;
    private double kI;
    private double kD;
    private double i;
    private double lastMeasurement = 0;
    private double lastSetpoint = 0;
    private double minIntegralErr = 0;
    private double maxIntegralErr = Double.POSITIVE_INFINITY;
    public boolean resetIntegralOnSetPointChange = false;
    private final ElapsedTime timer;
    private double lastTimestamp = 0;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        timer = new ElapsedTime();
        timer.reset();
    }

    public PIDController(PIDGains pidGains) {
        this(pidGains.kP, pidGains.kI, pidGains.kD);
    }

    public PIDController() {
        this(0, 0, 0);
    }

    public void setP(double kP) {
        this.kP = kP;
    }

    public void setI(double kI) {
        this.kI = kI;
    }

    public void setD(double kD) {
        this.kD = kD;
    }

    public void setGains(PIDGains pidGains) {
        this.kP = pidGains.kP;
        this.kI = pidGains.kI;
        this.kD = pidGains.kD;
    }

    public void setGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public PIDGains getGains() {
        return new PIDGains(kP, kI, kD);
    }

    /**
     * Defines the range in which the integral term will accumulate.  If the magnitude of the error is outside of this range, the integral will neither accumulate nor be added to the controller output.
     *
     * @param minErr The minimum magnitude the error needs to be for the integral to accumulate.
     * @param maxErr The maximum magnitude the error needs to be for the integral to accumulate.
     */
    public void setIntegralAccumulationRange(double minErr, double maxErr) {
        minIntegralErr = Math.abs(minErr);
        maxIntegralErr = Math.abs(maxErr);
    }

    private double elapsedTime() {
        double currentTime = timer.milliseconds();
        double elapsedTime = currentTime - lastTimestamp;
        lastTimestamp = currentTime;
        return elapsedTime > 200 ? 50 : elapsedTime;
    }

    public double calculate(double measurement, double setpoint) {
        double[] outputs = calculateTerms(measurement, setpoint);
        return outputs[0] + outputs[1] + outputs[2];
    }

    public double[] calculateTerms(double measurement, double setpoint) {
        double error = setpoint - measurement;
        double dt = elapsedTime();
        double p = kP * error;
        double d = kD * (measurement - lastMeasurement) / dt; // This does essentially the same thing as using de/dt. The only difference is that when the setpoint changes the output won't spike.
        lastMeasurement = measurement;
        if (lastSetpoint != setpoint && resetIntegralOnSetPointChange) {
            i = 0;
        }
        lastSetpoint = setpoint;
        if (Math.abs(error) > Math.abs(minIntegralErr) && Math.abs(error) < Math.abs(maxIntegralErr)) {
            i += kI * error * dt;
            return new double[]{p, i, d};
        } else {
            return new double[]{p, 0, d};
        }
    }

    public static class PIDGains {
        public final double kP;
        public final double kI;
        public final double kD;

        public PIDGains(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        @Override
        public String toString() {
            return "kp: " + kP + " kI: " + kI + " kD: " + kD;
        }
    }
}
