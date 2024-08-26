package com.acmerobotics.dashboard.canvas;

public class Stroke extends CanvasOp {
    private String color;

    public Stroke(String color) {
        super(Type.STROKE);

        this.color = color;
    }
}
