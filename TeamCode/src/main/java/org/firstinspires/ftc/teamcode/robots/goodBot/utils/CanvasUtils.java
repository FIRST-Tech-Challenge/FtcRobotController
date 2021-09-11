package org.firstinspires.ftc.teamcode.robots.goodBot.utils;

import com.acmerobotics.dashboard.canvas.Canvas;

public class CanvasUtils {
    public static Point toCanvasPoint(Point ugPoint) {
        return new Point(ugPoint.getY() * Constants.INCHES_PER_METER - 72, -ugPoint.getX() * Constants.INCHES_PER_METER);
    }

    public static void drawVector(Canvas canvas, Point ugCenter, double magnitude, double direction, String strokeColor) {
        canvas.setStroke(strokeColor);
        Point center = toCanvasPoint(ugCenter);
        canvas.strokeLine(
                center.getX(),
                center.getY(),
                center.getX() + magnitude * Math.sin(Math.toRadians(direction) + 0.5 * Math.PI),
                center.getY() + magnitude * Math.cos(Math.toRadians(direction) + 0.5 * Math.PI)
        );
    }

    public static class Point {
        private double x, y;
        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public void setX(double x) {
            this.x = x;
        }
        public void setY(double y) {
            this.y = y;
        }
        public double getX() {
            return x;
        }
        public double getY() {
            return y;
        }
    }
}
