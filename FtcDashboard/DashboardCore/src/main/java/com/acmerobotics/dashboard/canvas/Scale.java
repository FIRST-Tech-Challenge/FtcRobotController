package com.acmerobotics.dashboard.canvas;

public class Scale extends CanvasOp {
    private double scaleX;
    private double scaleY;

    public Scale(double scaleX, double scaleY) {
        super(Type.SCALE);

        this.scaleX = scaleX;
        this.scaleY = scaleY;
    }
}
