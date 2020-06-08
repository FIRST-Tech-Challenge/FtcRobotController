package org.firstinspires.ftc.teamcode.Trajectories;

public class CubicParametric implements Trajectory{

    public double a;
    public double b;
    public double c;
    public double d;

    public CubicParametric(double A, double B, double C, double D)
    {
        a = A;
        b = B;
        c = C;
        d = D;
    }

    public double getVal(double time)
    {
        return  (a * time * time * time) +
                (b * time * time) +
                (c * time) +
                (d);
    }

    public double getVelocity(double time)
    {
        return  (3 * a * time * time) +
                (2 * b * time) +
                (c);
    }

    public double getAcceleration(double time)
    {
        return  (6 * a * time) +
                (b);
    }

}
