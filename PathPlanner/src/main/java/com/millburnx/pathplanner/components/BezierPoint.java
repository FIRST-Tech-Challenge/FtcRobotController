package com.millburnx.pathplanner.components;

import java.awt.*;

public class BezierPoint {
    Point point;
    boolean isControlPoint;

    BezierPoint(Point point, boolean isControlPoint) {
        this.point = point;
        this.isControlPoint = isControlPoint;
    }
}