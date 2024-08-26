package com.acmerobotics.dashboard.canvas;

@SuppressWarnings({"checkstyle:MultipleVariableDeclarations", "checkstyle:EmptyLineSeparator"})
public class Spline extends CanvasOp {
    private double ax, bx, cx, dx, ex, fx;
    private double ay, by, cy, dy, ey, fy;

    public Spline(double ax, double bx, double cx, double dx, double ex, double fx,
                  double ay, double by, double cy, double dy, double ey, double fy) {
        super(Type.SPLINE);

        this.ax = ax;
        this.bx = bx;
        this.cx = cx;
        this.dx = dx;
        this.ex = ex;
        this.fx = fx;

        this.ay = ay;
        this.by = by;
        this.cy = cy;
        this.dy = dy;
        this.ey = ey;
        this.fy = fy;
    }
}
