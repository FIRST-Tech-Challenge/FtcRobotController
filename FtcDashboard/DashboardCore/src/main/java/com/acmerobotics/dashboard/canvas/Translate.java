package com.acmerobotics.dashboard.canvas;

public class Translate extends CanvasOp {
    private double x;
    private double y;

    public Translate(double x, double y) {
        super(Type.TRANSLATE);

        this.x = x;
        this.y = y;
    }
}
