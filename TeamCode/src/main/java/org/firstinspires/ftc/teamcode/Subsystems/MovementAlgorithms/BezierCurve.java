package org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms;
import java.util.ArrayList;
import java.util.List;

public class BezierCurve {
    // Bezier Curve implementation with improved numerical stability
        private List<Point> controlPoints;
        private int n;
        private double totalLength; // Cached arc length
        private List<Double> lengthTable; // For arc-length parameterization

        public BezierCurve(List<Point> controlPoints) {
            this.controlPoints = new ArrayList<>(controlPoints);
            this.n = controlPoints.size() - 1;
            precomputeLength();
        }

        private void precomputeLength() {
            lengthTable = new ArrayList<>();
            lengthTable.add(0.0);

            double length = 0;
            Point prevPoint = evaluate(0);

            for (int i = 1; i <= 100; i++) {
                double t = i / 100.0;
                Point currentPoint = evaluate(t);
                length += prevPoint.distanceTo(currentPoint);
                lengthTable.add(length);
                prevPoint = currentPoint;
            }
            totalLength = length;
        }

        private double binomialCoefficient(int n, int k) {
            if (k > n - k) k = n - k;
            if (k == 0 || k == n) return 1;

            double result = 1;
            for (int i = 0; i < k; i++) {
                result = result * (n - i) / (i + 1);
            }
            return result;
        }

        public Point evaluate(double t) {
            t = Math.max(0, Math.min(1, t)); // Clamp t to [0, 1]

            if (n == 0) return controlPoints.get(0).copy();

            double x = 0, y = 0;
            for (int i = 0; i <= n; i++) {
                double bernstein = binomialCoefficient(n, i) * Math.pow(t, i) * Math.pow(1 - t, n - i);
                x += bernstein * controlPoints.get(i).x;
                y += bernstein * controlPoints.get(i).y;
            }

            return new Point(x, y);
        }

        public Point derivative(double t) {
            if (n == 0) return new Point(0, 0);

            List<Point> derivativePoints = new ArrayList<>();
            for (int i = 0; i < n; i++) {
                double dx = n * (controlPoints.get(i + 1).x - controlPoints.get(i).x);
                double dy = n * (controlPoints.get(i + 1).y - controlPoints.get(i).y);
                derivativePoints.add(new Point(dx, dy));
            }

            BezierCurve derivativeCurve = new BezierCurve(derivativePoints);
            return derivativeCurve.evaluate(t);
        }

        public Point secondDerivative(double t) {
            if (n <= 1) return new Point(0, 0);

            List<Point> secondDerivativePoints = new ArrayList<>();
            for (int i = 0; i < n - 1; i++) {
                double d2x = n * (n - 1) * (
                        controlPoints.get(i + 2).x - 2 * controlPoints.get(i + 1).x + controlPoints.get(i).x
                );
                double d2y = n * (n - 1) * (
                        controlPoints.get(i + 2).y - 2 * controlPoints.get(i + 1).y + controlPoints.get(i).y
                );
                secondDerivativePoints.add(new Point(d2x, d2y));
            }

            if (secondDerivativePoints.size() == 1) {
                return secondDerivativePoints.get(0);
            }

            BezierCurve secondDerivativeCurve = new BezierCurve(secondDerivativePoints);
            return secondDerivativeCurve.evaluate(t);
        }

        public double curvature(double t) {
            Point firstDeriv = derivative(t);
            Point secondDeriv = secondDerivative(t);

            double crossProduct = firstDeriv.x * secondDeriv.y - firstDeriv.y * secondDeriv.x;
            double speedSquared = firstDeriv.x * firstDeriv.x + firstDeriv.y * firstDeriv.y;

            if (speedSquared < 1e-10) return 0;

            return Math.abs(crossProduct) / Math.pow(speedSquared, 1.5);
        }

        public double getTotalLength() {
            return totalLength;
        }
    }

