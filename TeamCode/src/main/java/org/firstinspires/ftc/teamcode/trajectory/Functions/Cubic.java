package org.firstinspires.ftc.teamcode.trajectory.Functions;

public class Cubic implements Function{

    private double a, b, c, d;

    public Cubic(double a, double b, double c, double d) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }

    public static Cubic cubicFromSpecs(double x0, double dx0, double x1, double dx1) {
        return new Cubic(2 * x0 + dx0 - 2 * x1 + dx1,
                         -3 * x0 - 2 * dx0 + 3 * x1 - dx1,
                            dx0,
                            x0);
    }

    public double getPoint(double x) {
        return (a*x*x*x) + (b*x*x) + (c*x) + d;
    }

    public double getDeriv(double x) {
        return (3*a*x*x) + (2*b*x) + c;
    }
}
