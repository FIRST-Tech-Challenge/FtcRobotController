package com.millburnx.pathplanner.components;

//import java.awt.*;

import com.millburnx.purePursuit.Utils.Point;

public class BezierPoint {
    private Point previousHandle;
    private Point anchor;
    private Point nextHandle;

    public BezierPoint(Point anchor, Point prev, Point next) {
        this.anchor = anchor;
        this.previousHandle = prev;
        this.nextHandle = next;
    }

    public Point getPreviousHandle() {
        return previousHandle;
    }

    public void setPreviousHandle(Point previousHandle) {
        this.previousHandle = previousHandle;
    }

    public Point getAnchor() {
        return anchor;
    }

    public void setAnchor(Point anchor) {
        this.anchor = anchor;
    }

    public Point getNextHandle() {
        return nextHandle;
    }

    public void setNextHandle(Point nextHandle) {
        this.nextHandle = nextHandle;
    }

    public boolean isAnchorNear(Point p, double threshold) {
        return anchor.distanceTo(p) < threshold;
    }

    public boolean isPrevNear(Point p, double threshold) {
        return previousHandle != null && previousHandle.distanceTo(p) < threshold;
    }

    public boolean isNextNear(Point p, double threshold) {
        return nextHandle != null && nextHandle.distanceTo(p) < threshold;
    }

    @Override
    public BezierPoint clone() {
        // Create new Points for anchor, handle1, and handle2 to ensure deep copy
        return new BezierPoint(
                anchor.copy(),
                previousHandle == null ? null : previousHandle.copy(),
                nextHandle == null ? null : nextHandle.copy()
        );
    }

    @Override
    public String toString() {
        return "BezierPoint{" +
                "previousHandle=" + previousHandle +
                ", anchor=" + anchor +
                ", nextHandle=" + nextHandle +
                '}';
    }
}
