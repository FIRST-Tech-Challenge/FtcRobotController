package org.firstinspires.ftc.teamcode;

public class PID {
    private double Kp;
    private double Ki;
    private double Kd;

    private double integralSum = 0.0;
    private double lastError = 0.0;

    private long startTime;

    private double dT = 20;

    public PID(double p, double i, double d) {
        Kp = p;
        Ki = i;
        Kd = d;
        startTime = System.currentTimeMillis();
    }

    // error is given by the user so that this class has multiple use cases
    public double getValue(double error) {

        double derivative = (error - lastError) / dT;

        // sum all the error over time
        integralSum += (error * dT);

        // returns this value which can differ depending on the use case of the class
        // eg. set motor power
        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        lastError = error;

        return out;
    }

    public void resetTimer() {
        startTime = System.currentTimeMillis();
    }

    public double getKp() {
        return Kp;
    }

    public double getKi() {
        return Ki;
    }

    public double getKd() {
        return Kd;
    }
}
