package org.firstinspires.ftc.teamcode.robots.reachRefactor.utils;

import com.acmerobotics.dashboard.canvas.Canvas;

import org.ejml.simple.SimpleMatrix;

public class CanvasUtils {
    public static void drawLine(Canvas canvas, SimpleMatrix p1, SimpleMatrix p2, String strokeColor) {
        canvas.setStroke(strokeColor);
        canvas.strokeLine(
                p1.get(0),
                p1.get(1),
                p2.get(0),
                p2.get(1)
        );
    }

    public static void drawDottedLine(Canvas canvas, SimpleMatrix p1, SimpleMatrix p2, String strokeColor, double dashLength) {
        canvas.setStroke(strokeColor);

        double distance = p2.minus(p1).normF();
        double steps = distance / dashLength;

        SimpleMatrix stepSize = p2.minus(p1).divide(steps);
        for(double i = 0; i < steps - 1; i += dashLength * 2) {
            drawLine(canvas, p1.plus(stepSize.scale(i)), p1.plus(stepSize.scale(i + dashLength)), strokeColor);
        }
    }

    public static void drawVector(Canvas canvas, SimpleMatrix center, double magnitude, double direction, String strokeColor) {
        canvas.setStroke(strokeColor);
        canvas.strokeLine(
                center.get(0),
                center.get(0),
                center.get(0) + magnitude * Math.cos(direction),
                center.get(1) + magnitude * Math.sin(direction)
        );
    }
}
