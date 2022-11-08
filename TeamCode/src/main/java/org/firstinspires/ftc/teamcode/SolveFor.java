package org.firstinspires.ftc.teamcode;

public class SolveFor {

    private double m = 0;
    private double c = 0;

    public SolveFor(double X1, double Y1, double X2, double Y2) {
        m = (Y2-Y1)/(X2-X1);
        c = Y1 - (m*X1);
    }

    public double getC() {
        return c;
    }

    public double getM() {
        return m;
    }
}
