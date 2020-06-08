package org.firstinspires.ftc.teamcode.Trajectories;

public interface Trajectory {

    public double getVal(double time);
    public double getVelocity(double time);
    public double getAcceleration(double time);

}
