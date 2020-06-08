package org.firstinspires.ftc.teamcode.Trajectories;

public interface Trajectory2D {

    public double totalTime = 0;

    public double getValX(double time);
    public double getValY(double time);

    public double getVelocityX(double time);
    public double getVelocityY(double time);

    public double getAccelerationX(double time);
    public double getAccelerationY(double time);

}
