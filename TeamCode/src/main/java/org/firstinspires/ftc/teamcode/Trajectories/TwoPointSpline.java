package org.firstinspires.ftc.teamcode.Trajectories;

import org.firstinspires.ftc.teamcode.Controllables.Location;

public class TwoPointSpline implements Trajectory2D {

    private CubicParametric xSpline;
    private CubicParametric ySpline;
    public double totalTime;

    public TwoPointSpline(Location start, Location end, double totalTime)
    {
        xSpline = new CubicParametric(((2 * start.xPos) - (2 * end.xPos)) * Math.pow(totalTime, -3),
                                      ((3 * end.xPos) - (3 * start.xPos)) * Math.pow(totalTime, -2),
                                      0,
                                         start.xPos);
        ySpline = new CubicParametric(((2 * start.yPos) - (2 * end.yPos)) * Math.pow(totalTime, -3),
                                      ((3 * end.yPos) - (3 * start.yPos)) * Math.pow(totalTime, -2),
                                      0,
                                         start.yPos);
        this.totalTime = totalTime;
    }

    @Override
    public double getValX(double time) {
        return xSpline.getVal(time);
    }

    @Override
    public double getValY(double time) {
        return ySpline.getVal(time);
    }

    @Override
    public double getVelocityX(double time) {
        return xSpline.getVelocity(time);
    }

    @Override
    public double getVelocityY(double time) {
        return ySpline.getVelocity(time);
    }

    @Override
    public double getAccelerationX(double time) {
        return xSpline.getAcceleration(time);
    }

    @Override
    public double getAccelerationY(double time) {
        return ySpline.getAcceleration(time);
    }
}
