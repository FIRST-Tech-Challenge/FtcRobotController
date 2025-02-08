package com.kalipsorobotics.localization;



import android.graphics.PointF;

import com.kalipsorobotics.math.Position;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.opencv.core.Point;


public class KalmanFilter {
    // Kalman filter parameters for x and y coordinates
    private double q; // Process noise covariance
    private double r; // Measurement noise covariance
    private double t; // Heading Noise
    private double x; // Estimated value (x)
    private double y; // Estimated value (y)
    private double h; // Estimated Value (heading)
    private double pX; // Estimation error covariance (x)
    private double pY; // Estimation error covariance (y)
    private double pH;
    private double kX; // Kalman gain (x)
    private double kY; // Kalman gain (y)
    private double kH;
    // Constructor
    public KalmanFilter(double processNoise, double measurementNoise, double headingNoise) {
        this.q = processNoise;
        this.r = measurementNoise;
        this.t = headingNoise;
        this.pX = 1; // Initial estimation error covariance (x)
        this.pY = 1; // Initial estimation error covariance (y)
        this.pH = 1;
        this.x = 0; // Initial estimate (x)
        this.y = 0; // Initial estimate (y)
        this.h = 0;
    }
    public Position update(Position measurement) {
        // Update x-coordinate
        kX = pX / (pX + r);
        x = x + kX * (measurement.getX() - x);
        pX = (1 - kX) * pX + q;

        // Update y-coordinate
        kY = pY / (pY + r);
        y = y + kY * (measurement.getY() - y);
        pY = (1 - kY) * pY + q;

        kH = pH / (pH + t);
        h = h + kH * (measurement.getTheta() - h);
        pH = (1 - kH) * pH + t;

        return new Position(x, y, h);
    }

    // Update and get the filtered point
    public Point update(Point measurement) {
        // Update x-coordinate
        kX = pX / (pX + r);
        x = x + kX * (measurement.x - x);
        pX = (1 - kX) * pX + q;

        // Update y-coordinate
        kY = pY / (pY + r);
        y = y + kY * (measurement.y - y);
        pY = (1 - kY) * pY + q;

        // Return the filtered point
        return new Point(x, y); // Return a PointF object
    }

    // Reset the filter
    public void reset() {
        this.x = 0;
        this.y = 0;
        this.pX = 1;
        this.pY = 1;
    }

    // FTC-specific test method
    public static void testFilter(Telemetry telemetry) {
        // Initialize the Kalman filter
        KalmanFilter kalmanFilter = new KalmanFilter(0.01, 1, 0);

        // Simulate noisy measurements
        Point[] measurements = {
                new Point(100, 200),
                new Point(102, 198),
                new Point(105, 203),
                new Point(108, 200),
                new Point(110, 202)
        };

        // Process the measurements and log the results
        for (Point measurement : measurements) {
            Point filteredPoint = kalmanFilter.update(measurement);
            telemetry.addData("Measurement", "x: %d, y: %d", measurement.x, measurement.y);
            telemetry.addData("Filtered", "x: %d, y: %d", filteredPoint.x, filteredPoint.y);
            telemetry.update();
        }
    }
}
