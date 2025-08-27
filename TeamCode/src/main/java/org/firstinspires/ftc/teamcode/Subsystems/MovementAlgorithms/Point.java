package org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms;

public class Point {
        public double x, y;

        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public Point copy() {
            return new Point(x, y);
        }

        public double distanceTo(Point other) {
            return Math.sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
        }

        public Point subtract(Point other) {
            return new Point(x - other.x, y - other.y);
        }

        public Point add(Point other) {
            return new Point(x + other.x, y + other.y);
        }

        public double magnitude() {
            return Math.sqrt(x * x + y * y);
        }

        public Point normalize() {
            double mag = magnitude();
            return mag > 1e-10 ? new Point(x / mag, y / mag) : new Point(0, 0);
        }
    }


