package com.acmerobotics.dashboard.canvas;

public class Polygon extends CanvasOp {
    private double[] xPoints;
    private double[] yPoints;
    private boolean stroke;

    public Polygon(double[] xPoints, double[] yPoints, boolean stroke) {
        super(Type.POLYGON);

        this.xPoints = xPoints;
        this.yPoints = yPoints;
        this.stroke = stroke;
    }
}
