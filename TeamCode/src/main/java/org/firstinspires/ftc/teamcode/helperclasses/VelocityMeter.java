package org.firstinspires.ftc.teamcode.helperclasses;

import org.firstinspires.ftc.teamcode.competition.Hardware;

public class VelocityMeter implements Runnable {

    private Hardware robot;

    public static double velocity;
    public static double angularVelocity;
    public boolean stop = false;

    public VelocityMeter(Hardware robot) {
        this.robot = robot;
    }

    /**
     * Calculates the robots velocity based on odometry
     * Not currently implemented
     * Do not use for velocity measurements for calculations that reilly on accurate real time data
     */
    public void run() {
    }
}
