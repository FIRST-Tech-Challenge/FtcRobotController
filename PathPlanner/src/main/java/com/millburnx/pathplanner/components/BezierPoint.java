package com.millburnx.pathplanner.components;

import java.awt.*;

public class BezierPoint {
    private Point anchor;   // Main point
    private Point handle1;  // First control point for the Bézier curve
    private Point handle2;  // Second control point for the Bézier curve (optional)

    public BezierPoint(Point anchor, Point handle1, Point handle2) {
        this.anchor = anchor;
        this.handle1 = handle1;
        this.handle2 = handle2;
    }

    public Point getAnchor() {
        return anchor;
    }

    public Point getHandle1() {
        return handle1;
    }

    public Point getHandle2() {
        return handle2;
    }

    public void setAnchor(Point anchor) {
        this.anchor = anchor;
    }

    public void setHandle1(Point handle1) {
        this.handle1 = handle1;
    }

    public void setHandle2(Point handle2) {
        this.handle2 = handle2;
    }

    public boolean isAnchorNear(Point p) {
        return anchor.distance(p) < 15;
    }

    public boolean isHandle1Near(Point p) {
        return handle1 != null && handle1.distance(p) < 10;
    }

    public boolean isHandle2Near(Point p) {
        return handle2 != null && handle2.distance(p) < 10;
    }

    @Override
    public BezierPoint clone() {
        // Create new Points for anchor, handle1, and handle2 to ensure deep copy
        return new BezierPoint(
                new Point(this.anchor),
                this.handle1 != null ? new Point(this.handle1) : null,
                this.handle2 != null ? new Point(this.handle2) : null
        );
    }
}
