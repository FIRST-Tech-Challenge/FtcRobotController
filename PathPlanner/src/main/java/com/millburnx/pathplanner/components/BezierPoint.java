package com.millburnx.pathplanner.components;

//import java.awt.*;

import com.millburnx.utils.Vec2d;

public class BezierPoint {
    private Vec2d previousHandle;
    private Vec2d anchor;
    private Vec2d nextHandle;

    public BezierPoint(Vec2d anchor, Vec2d prev, Vec2d next) {
        this.anchor = anchor;
        this.previousHandle = prev;
        this.nextHandle = next;
    }

    public Vec2d getPreviousHandle() {
        return previousHandle;
    }

    public void setPreviousHandle(Vec2d previousHandle) {
        this.previousHandle = previousHandle;
    }

    public Vec2d getAnchor() {
        return anchor;
    }

    public void setAnchor(Vec2d anchor) {
        this.anchor = anchor;
    }

    public Vec2d getNextHandle() {
        return nextHandle;
    }

    public void setNextHandle(Vec2d nextHandle) {
        this.nextHandle = nextHandle;
    }

    public boolean isAnchorNear(Vec2d p, double threshold) {
        return anchor.distanceTo(p) < threshold;
    }

    public boolean isPrevNear(Vec2d p, double threshold) {
        return previousHandle != null && previousHandle.distanceTo(p) < threshold;
    }

    public boolean isNextNear(Vec2d p, double threshold) {
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
