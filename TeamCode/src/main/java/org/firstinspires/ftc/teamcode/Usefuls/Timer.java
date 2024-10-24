package org.firstinspires.ftc.teamcode.Usefuls;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Timer {
    private double t = 0.0;
    private double dt = 0.0;
    private ElapsedTime elapsedTime;

    public Timer() {
        this.elapsedTime = new ElapsedTime();
    }

    public double getT() { return this.t; }
    public double getDt() { return this.dt; }

    public void update() {
        double t = this.elapsedTime.seconds();
        double dt = t - this.t;

        this.t = t;
        this.dt = dt;
    }
}