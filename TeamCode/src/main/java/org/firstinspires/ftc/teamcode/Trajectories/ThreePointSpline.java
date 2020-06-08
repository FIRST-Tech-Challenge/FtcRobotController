package org.firstinspires.ftc.teamcode.Trajectories;

import org.firstinspires.ftc.teamcode.Controllables.Location;

public class ThreePointSpline implements Trajectory2D {

    // These values are used in the linear algebra to define the rotation of the space
    private double resultX1;    // X value of where the j^ vector lands
    private double resultY1;    // Y value of where the j^ vector lands

    private double resultX2;    // X value of where the i^ vector lands
    private double resultY2;    // Y value of where the i^ vector lands


    private double inverseResultX1;
    private double inverseResultY1;

    private double inverseResultX2;
    private double inverseResultY2;


    // Values defining the  the
    private CubicParametric normalXFunction1;
    private CubicParametric normalYFunction1;

    private CubicParametric normalXFunction2;
    private CubicParametric normalYFunction2;

    public double totalTime;
    private double halfTotalTime;


    public ThreePointSpline(Location point1, Location point2, Location point3, double totalTime, double c)
    {
        //Math to figure out where the basis vectors land
        //Look under "Basis Vectors" in the Desmos Graph
        resultX1 =  point3.xPos / ((point3.xPos * point3.xPos) + (point3.yPos * point3.yPos));
        resultY1 =  point3.yPos / ((point3.xPos * point3.xPos) + (point3.yPos * point3.yPos));

        resultX2 =  point3.yPos / ((point3.xPos * point3.xPos) + (point3.yPos * point3.yPos));
        resultY2 = -point3.xPos / ((point3.xPos * point3.xPos) + (point3.yPos * point3.yPos));

        double rotatedX2 = (point2.xPos * resultX1) + (point2.yPos * resultX2);
        double rotatedY2 = -(point2.xPos * resultY1) + (point2.yPos * resultY2);

        double a1 = c * Math.pow(Math.abs((.5 - rotatedX2)), 5/4);
        double b1 = rotatedX2 - a1;
        double a2 = ((-3/2) * a1) + (.5 * (1 - b1));
        double b2 = rotatedX2 - a2 - 1;

        normalXFunction1 = new CubicParametric(a1, b1, 0, 0);
        normalXFunction2 = new CubicParametric(a2, b2, 0, 1);

        a1 = 2 * (0 - rotatedY2);
        b1 = rotatedY2 - a1;
        a2 = 2 * (0 - rotatedY2);
        b2 = rotatedY2 - a2;

        normalYFunction1 = new CubicParametric(a1, b1, 0, 0);
        normalYFunction2 = new CubicParametric(a2, b2, 0, 0);

        double g = 1 / ((resultX1 * resultY2) - (resultX2 * resultY1));
        inverseResultX1 = resultY2 * g;
        inverseResultY1 = -resultY1 * g;
        inverseResultX2 = resultX2 * g;
        inverseResultY2 = -resultX1 * g;

        this.totalTime = totalTime;
        halfTotalTime = totalTime / 2;
    }

    @Override
    public double getValX(double time)
    {
        time = time / halfTotalTime;
        if (time < 1)
        {
            return (normalXFunction1.getVal(time) * inverseResultX1) + (normalYFunction1.getVal(time) * inverseResultX2);
        }
        else
        {
            return (normalXFunction2.getVal(time) * inverseResultX1) + (normalYFunction2.getVal(time) * inverseResultX2);
        }
    }

    @Override
    public double getValY(double time)
    {
        if (time < halfTotalTime)
        {
            return (normalXFunction1.getVal(time) * inverseResultY1) + (normalYFunction1.getVal(time) * inverseResultY2);
        }
        else
        {
            return (normalXFunction2.getVal(time) * inverseResultY1) + (normalYFunction2.getVal(time) * inverseResultY2);
        }
    }

    // Unsure how exactly to deal with
    @Override
    public double getVelocityX(double time)
    {
        if (time < halfTotalTime)
        {
            return (normalXFunction1.getVelocity(time) * inverseResultX1) + (normalYFunction1.getVelocity(time) * inverseResultX2);
        }
        else
        {
            return (normalXFunction2.getVelocity(time) * inverseResultX1) + (normalYFunction2.getVelocity(time) * inverseResultX2);
        }
    }

    // Unsure how exactly to deal with
    @Override
    public double getVelocityY(double time)
    {
        if (time < halfTotalTime)
        {
            return (normalXFunction1.getVelocity(time) * inverseResultY1) + (normalYFunction1.getVelocity(time) * inverseResultY2);
        }
        else
        {
            return (normalXFunction2.getVelocity(time) * inverseResultY1) + (normalYFunction2.getVelocity(time) * inverseResultY2);
        }
    }

    // Unsure how exactly to deal with
    @Override
    public double getAccelerationX(double time)
    {
        if (time < halfTotalTime)
        {
            return (normalXFunction1.getAcceleration(time) * inverseResultX1) + (normalYFunction1.getAcceleration(time) * inverseResultX2);
        }
        else
        {
            return (normalXFunction2.getAcceleration(time) * inverseResultX1) + (normalYFunction2.getAcceleration(time) * inverseResultX2);
        }
    }

    // Unsure how exactly to deal with
    @Override
    public double getAccelerationY(double time)
    {
        if (time < halfTotalTime)
        {
            return (normalXFunction1.getAcceleration(time) * inverseResultY1) + (normalYFunction1.getAcceleration(time) * inverseResultY2);
        }
        else
        {
            return (normalXFunction2.getAcceleration(time) * inverseResultY1) + (normalYFunction2.getAcceleration(time) * inverseResultY2);
        }
    }
}
