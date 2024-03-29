package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

public class AprilTagProcessorImplCallback extends AprilTagProcessorImpl {
    // The following coordinates assume field bottom left is coordinate 0, 0
    // Backdrop AprilTag 1, left backdrop left april tag
    // Center tag - 6"
    // 5x23" + 5x0.75" + 11.5"
    public static final FieldCoordinate APRIL_TAG_1 = new FieldCoordinate(29.25, 130.25, Math.toRadians(90.0));
    // Backdrop AprilTag 2, left backdrop center april tag
    // 23" + 0.75" + 11.5"
    public static final FieldCoordinate APRIL_TAG_2 = new FieldCoordinate(35.25, 130.25, Math.toRadians(90.0));
    // Backdrop AprilTag 3, left backdrop right april tag
    // Center tag + 6"
    public static final FieldCoordinate APRIL_TAG_3 = new FieldCoordinate(41.25, 130.25, Math.toRadians(90.0));
    // Backdrop AprilTag 4, right backdrop left april tag
    // Center tag - 6"
    public static final FieldCoordinate APRIL_TAG_4 = new FieldCoordinate(100.5, 130.25, Math.toRadians(90.0));
    // Backdrop AprilTag 5, right backdrop center april tag
    // 4x23" + 4x0.75" + 11.5"
    public static final FieldCoordinate APRIL_TAG_5 = new FieldCoordinate(106.5, 130.25, Math.toRadians(90.0));
    // Backdrop AprilTag 6, right backdrop right april tag
    // Center tag + 6"
    public static final FieldCoordinate APRIL_TAG_6 = new FieldCoordinate(112.5, 130.25, Math.toRadians(90.0));
    // Left side stack images
    // 23" + 0.75" + 11.5"
    public static final FieldCoordinate APRIL_TAG_7 = new FieldCoordinate(35.25, 0.0, Math.toRadians(270.0));
    public static final FieldCoordinate APRIL_TAG_8 = new FieldCoordinate(35.25, 0.0, Math.toRadians(270.0));
    // Right side stack images
    // 4x23" + 4x0.75" + 11.5"
    public static final FieldCoordinate APRIL_TAG_9 = new FieldCoordinate(106.5, 0.0, Math.toRadians(270.0));
    public static final FieldCoordinate APRIL_TAG_10 = new FieldCoordinate(106.5, 0.0, Math.toRadians(270.0));
    protected FieldCoordinate robotPosition;
    public AprilTagProcessorImplCallback(double fx, double fy, double cx, double cy, DistanceUnit outputUnitsLength,
                                         AngleUnit outputUnitsAngle, AprilTagLibrary tagLibrary, boolean drawAxes,
                                         boolean drawCube, boolean drawOutline, boolean drawTagID, TagFamily tagFamily,
                                         int threads, FieldCoordinate robot) {
        super(fx, fy, cx, cy, outputUnitsLength, outputUnitsAngle, tagLibrary, drawAxes, drawCube, drawOutline, drawTagID, tagFamily, threads);
        robotPosition = robot;
    }
    // This is called by the SDK for every frame to get april tag detections. Call the original
    // function to get the detections, then use those detections, if any, to update the robot
    // location.
    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Object localDetections;
        localDetections = super.processFrame(input, captureTimeNanos);
        updateLocation(super.getFreshDetections());
        return localDetections;
    }
    // Go through all the april tag detections and update the robot location.
    public void updateLocation(ArrayList<AprilTagDetection> detections) {
        if(detections != null) {
            List<FieldCoordinate> detectionCoordinates = new ArrayList<>();
            FieldCoordinate detectionCoord;
            for(AprilTagDetection detection : detections) {
                detectionCoord = calculateCoordinatesFromDetection(detection);
                if(detectionCoord != null) { detectionCoordinates.add(detectionCoord); }
            }
            int detectionCount = detectionCoordinates.size();
            if(detectionCount > 0) {
                double sumX = 0.0;
                double sumY = 0.0;
                double sumAngleRadians = 0.0;
                for(FieldCoordinate detection : detectionCoordinates) {
                    sumX += detection.getX();
                    sumY += detection.getY();
                    sumAngleRadians += detection.getAngleRadians();
                }
                double x = sumX / detectionCount;
                double y = sumY / detectionCount;
                double angleRadians = sumAngleRadians / detectionCount;
                synchronized(robotPosition.lock) {
                    robotPosition.setLocation(x, y, angleRadians);
                }
            }
        }
    }
    // Calculate a location based on an april tag detection
    public FieldCoordinate calculateCoordinatesFromDetection(AprilTagDetection detection) {
        FieldCoordinate result = null;
        // Throw out detections that we believe will cause more harm than good.
        // TODO: update this to real data, and not just a placeholder and consider what all we want
        // to use to invalidate a detection.
        if(detection.ftcPose.yaw < 20.0) {
            // TODO: Do the math to convert an april tag detection to a coordinate
            double errorX = detection.ftcPose.range * Math.cos( Math.toRadians(detection.ftcPose.bearing) );
            double errorY = detection.ftcPose.range * Math.sin( Math.toRadians(detection.ftcPose.bearing) );
            double errorAngleRadians = Math.toRadians(detection.ftcPose.yaw);
            switch(detection.id) {
                case 1:
                    errorX = APRIL_TAG_1.getX() - errorX;
                    errorY = APRIL_TAG_1.getY() - errorY;
                    errorAngleRadians = APRIL_TAG_1.getAngleRadians() - errorAngleRadians;
                    result = new FieldCoordinate(errorX, errorY, errorAngleRadians);
                    break;
                case 2:
                    errorX = APRIL_TAG_2.getX() - errorX;
                    errorY = APRIL_TAG_2.getY() - errorY;
                    errorAngleRadians = APRIL_TAG_2.getAngleRadians() - errorAngleRadians;
                    result = new FieldCoordinate(errorX, errorY, errorAngleRadians);
                    break;
                case 3:
                    errorX = APRIL_TAG_3.getX() - errorX;
                    errorY = APRIL_TAG_3.getY() - errorY;
                    errorAngleRadians = APRIL_TAG_3.getAngleRadians() - errorAngleRadians;
                    result = new FieldCoordinate(errorX, errorY, errorAngleRadians);
                    break;
                case 4:
                    errorX = APRIL_TAG_4.getX() - errorX;
                    errorY = APRIL_TAG_4.getY() - errorY;
                    errorAngleRadians = APRIL_TAG_4.getAngleRadians() - errorAngleRadians;
                    result = new FieldCoordinate(errorX, errorY, errorAngleRadians);
                    break;
                case 5:
                    errorX = APRIL_TAG_5.getX() - errorX;
                    errorY = APRIL_TAG_5.getY() - errorY;
                    errorAngleRadians = APRIL_TAG_5.getAngleRadians() - errorAngleRadians;
                    result = new FieldCoordinate(errorX, errorY, errorAngleRadians);
                    break;
                case 6:
                    errorX = APRIL_TAG_6.getX() - errorX;
                    errorY = APRIL_TAG_6.getY() - errorY;
                    errorAngleRadians = APRIL_TAG_6.getAngleRadians() - errorAngleRadians;
                    result = new FieldCoordinate(errorX, errorY, errorAngleRadians);
                    break;
                case 7:
                    errorX = APRIL_TAG_7.getX() - errorX;
                    errorY = APRIL_TAG_7.getY() - errorY;
                    errorAngleRadians = APRIL_TAG_7.getAngleRadians() - errorAngleRadians;
                    result = new FieldCoordinate(errorX, errorY, errorAngleRadians);
                    break;
                case 8:
                    errorX = APRIL_TAG_8.getX() - errorX;
                    errorY = APRIL_TAG_8.getY() - errorY;
                    errorAngleRadians = APRIL_TAG_8.getAngleRadians() - errorAngleRadians;
                    result = new FieldCoordinate(errorX, errorY, errorAngleRadians);
                    break;
                default:
                    result = null;
            }
        }
        return result;
    }
}
