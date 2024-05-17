package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AprilTagProcessorImplCallback extends AprilTagProcessorImpl {
    //    int tagId, FieldCoordinate location, double range, double angleDeg
    // The following coordinates assume field bottom left is coordinate 0, 0
    // Backdrop AprilTag 1, left backdrop left april tag
    // Center tag - 6"
    // 5x23" + 5x0.75" + 11.5"
    public static final AprilTagTarget APRIL_TAG_1 = new AprilTagTarget(1, new FieldCoordinate(29.25, 130.25, 270.0), 18.0, 20.0);
    // Backdrop AprilTag 2, left backdrop center april tag
    // 23" + 0.75" + 11.5"
    public static final AprilTagTarget APRIL_TAG_2 = new AprilTagTarget(2, new FieldCoordinate(35.25, 130.25, 270.0), 18.0, 20.0);
    // Backdrop AprilTag 3, left backdrop right april tag
    // Center tag + 6"
    public static final AprilTagTarget APRIL_TAG_3 = new AprilTagTarget(3, new FieldCoordinate(41.25, 130.25, 270.0), 18.0, 20.0);
    // Backdrop AprilTag 4, right backdrop left april tag
    // Center tag - 6"
    public static final AprilTagTarget APRIL_TAG_4 = new AprilTagTarget(4, new FieldCoordinate(100.5, 130.25, 270.0), 18.0, 20.0);
    // Backdrop AprilTag 5, right backdrop center april tag
    // 4x23" + 4x0.75" + 11.5"
    public static final AprilTagTarget APRIL_TAG_5 = new AprilTagTarget(5, new FieldCoordinate(106.5, 130.25, 270.0), 18.0, 20.0);
    // Backdrop AprilTag 6, right backdrop right april tag
    // Center tag + 6"
    public static final AprilTagTarget APRIL_TAG_6 = new AprilTagTarget(6, new FieldCoordinate(112.5, 130.25, 270.0), 18.0, 20.0);
    // Left side stack images
    // 23" + 0.75" + 11.5"
    // Y Min
//    public static final FieldCoordinate APRIL_TAG_7 = new FieldCoordinate(111.5, 0.0, Math.toRadians(270.0));
//    public static final FieldCoordinate APRIL_TAG_8 = new FieldCoordinate(106.0, 0.0, Math.toRadians(270.0));
    // X Max
//    public static final FieldCoordinate APRIL_TAG_7 = new FieldCoordinate(141.75, 32.5, Math.toRadians(0.0));
//    public static final FieldCoordinate APRIL_TAG_8 = new FieldCoordinate(141.75, 27.0, Math.toRadians(0.0));
    // X Min
    public static final AprilTagTarget APRIL_TAG_7 = new AprilTagTarget(7, new FieldCoordinate(141.75, 27.0, 180.0), 36.0, 20.0);
    public static final AprilTagTarget APRIL_TAG_8 = new AprilTagTarget(8, new FieldCoordinate(106.0, 0.0, 90.0), 18.0, 20.0);
    // Right side stack images
    // 4x23" + 4x0.75" + 11.5"
    public static final AprilTagTarget APRIL_TAG_9 = new AprilTagTarget(9, new FieldCoordinate(34.75, 0.0, 90.0), 18.0, 20.0);
    public static final AprilTagTarget APRIL_TAG_10 = new AprilTagTarget(10, new FieldCoordinate(0.0, 29.5, 0.0), 36.0, 20.0);
    protected Map<Integer, AprilTagTarget> aprilTags;
    protected FieldCoordinate robotPosition;
    public final static Object positionLock = new Object();
    protected Telemetry telemetry;
    public AprilTagProcessorImplCallback(double fx, double fy, double cx, double cy, DistanceUnit outputUnitsLength,
                                         AngleUnit outputUnitsAngle, AprilTagLibrary tagLibrary, boolean drawAxes,
                                         boolean drawCube, boolean drawOutline, boolean drawTagID, TagFamily tagFamily,
                                         int threads, boolean suppressCalibrationWarnings, FieldCoordinate robot,
                                         Telemetry telemetry) {
        super(fx, fy, cx, cy, outputUnitsLength, outputUnitsAngle, tagLibrary, drawAxes, drawCube, drawOutline, drawTagID, tagFamily, threads, suppressCalibrationWarnings);
        aprilTags = new HashMap<>();
        aprilTags.put(APRIL_TAG_1.aprilTagId, APRIL_TAG_1);
        aprilTags.put(APRIL_TAG_2.aprilTagId, APRIL_TAG_2);
        aprilTags.put(APRIL_TAG_3.aprilTagId, APRIL_TAG_3);
        aprilTags.put(APRIL_TAG_4.aprilTagId, APRIL_TAG_4);
        aprilTags.put(APRIL_TAG_5.aprilTagId, APRIL_TAG_5);
        aprilTags.put(APRIL_TAG_6.aprilTagId, APRIL_TAG_6);
        aprilTags.put(APRIL_TAG_7.aprilTagId, APRIL_TAG_7);
        aprilTags.put(APRIL_TAG_8.aprilTagId, APRIL_TAG_8);
        aprilTags.put(APRIL_TAG_9.aprilTagId, APRIL_TAG_9);
        aprilTags.put(APRIL_TAG_10.aprilTagId, APRIL_TAG_10);
        robotPosition = robot;
        this.telemetry = telemetry;
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
                if((detection != null) && (detection.id >= 1) && (detection.id <= 10)) {
                    detectionCoord = aprilTags.get(detection.id).calculateCoordinatesFromDetection(detection);
                    if(detectionCoord != null) { detectionCoordinates.add(detectionCoord); }
                }
            }
            int detectionCount = detectionCoordinates.size();
            if(detectionCount > 0) {
                double sumX = 0.0;
                double sumY = 0.0;
                double sumAngleDegrees = 0.0;
                for(FieldCoordinate detection : detectionCoordinates) {
                    sumX += detection.getX();
                    sumY += detection.getY();
                    sumAngleDegrees += detection.getAngleDegrees();
                }
                double x = sumX / detectionCount;
                double y = sumY / detectionCount;
                double angleDegrees = sumAngleDegrees / detectionCount;
                synchronized(positionLock) {
                    robotPosition.setLocation(x, y, angleDegrees);
                }
            }
        }
    }
}
