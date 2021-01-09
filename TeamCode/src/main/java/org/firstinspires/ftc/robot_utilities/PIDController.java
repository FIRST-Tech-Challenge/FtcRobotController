package org.firstinspires.ftc.robot_utilities;

import java.util.LinkedList;
import java.util.Queue;

public class PIDController {
    private double encoderValue = 0;

    private final int MAX_ROTATIONS_6000 = 2800;
    public double kp, ki, kd;
    private double measuredSpeedAvg = 0;
    Queue<Double> measuredSpeeds = new LinkedList<>();

    private double errorTolerance_p = 0.05;
    private double errorTolerance_v = Double.POSITIVE_INFINITY;

    private double lastTimeStamp;
    private double period;

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;

        this.lastTimeStamp = (double)System.nanoTime() / 1E9;
    }



    public double getUpdate(double velocity, double targetSpeed) {
        double lastSpeed = measuredSpeedAvg;
        if(measuredSpeeds.size() > 10) {
            measuredSpeedAvg = (measuredSpeedAvg * 10 - measuredSpeeds.poll() + velocity) / 10;
        } else {
            measuredSpeedAvg = (measuredSpeedAvg * measuredSpeeds.size() + velocity) / (measuredSpeeds.size() + 1);
        }
        measuredSpeeds.add(velocity);

        double currentTimeStamp = (double)System.nanoTime() / 1E9;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        return 0;


    }

    /*



     */
}
