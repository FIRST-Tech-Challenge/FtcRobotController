package org.firstinspires.ftc.teamcode.robots.reachRefactor.utils;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class CanvasUtils {

    public static void drawLine(Canvas canvas, Vector2d p1, Vector2d p2) {
        canvas.strokeLine(
                p1.getX(),
                p1.getY(),
                p2.getX(),
                p2.getY()
        );
    }

    public static void drawDottedLine(Canvas canvas, Vector2d p1, Vector2d p2, double dashLength) {
        double distance = p1.distTo(p2);
        double steps = distance / dashLength;

        Vector2d stepSize = p2.minus(p1).div(steps);
        for(double i = 0; i < steps - 1; i += dashLength * 2) {
            drawLine(canvas, p1.plus(stepSize.times(i)), p1.plus(stepSize.times(i + dashLength)));
        }
    }
}
