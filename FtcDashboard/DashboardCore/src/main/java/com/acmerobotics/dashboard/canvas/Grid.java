package com.acmerobotics.dashboard.canvas;

public class Grid extends CanvasOp {

    private double x, y;
    private double width, height;
    private int numTicksX, numTicksY;
    private double theta, pivotX, pivotY;
    private boolean usePageFrame;

    public Grid(double x, double y, double width, double height, int numTicksX, int numTicksY,
                double theta, double pivotX, double pivotY, boolean usePageFrame) {
        super(Type.GRID);

        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
        this.numTicksX = numTicksX;
        this.numTicksY = numTicksY;
        this.theta = theta;
        this.pivotX = pivotX;
        this.pivotY = pivotY;
        this.usePageFrame = usePageFrame;
    }
}
