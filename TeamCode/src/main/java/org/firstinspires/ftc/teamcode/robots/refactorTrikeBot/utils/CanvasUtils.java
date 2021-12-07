package org.firstinspires.ftc.teamcode.robots.refactorTrikeBot.utils;

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
}
