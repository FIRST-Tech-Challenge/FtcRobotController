package org.firstinspires.ftc.team6220_PowerPlay;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class AprilTagDetectionPipeline extends OpenCvPipeline {
    private static final Scalar blue = new Scalar(7, 197, 235, 255);
    private static final Scalar red = new Scalar(255, 0, 0, 255);
    private static final Scalar green = new Scalar(0, 255, 0, 255);
    private static final Scalar white = new Scalar(255, 255, 255, 255);

    private final Mat gray = new Mat();
    private final Object detectionsUpdateSync = new Object();

    // units are pixels
    // calibration is for Logitech C920 webcam at 1920 x 1080
    private final double fx = 1385.920; // focal length x
    private final double fy = 1385.920; // focal length y
    private final double cx = 951.982; // camera principal point x
    private final double cy = 534.084; // camera principal point y

    // units are meters
    private final double tagSize = 0.03429;
    private final Object decimationSync = new Object();

    Mat cameraMatrix;

    private long aprilTagPointer;

    private ArrayList<AprilTagDetection> detections = new ArrayList<>();
    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();

    private float decimation;
    private boolean needToSetDecimation;

    public AprilTagDetectionPipeline() {
        constructMatrix();
        aprilTagPointer = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    @Override
    protected void finalize() {
        if (aprilTagPointer != 0) {
            AprilTagDetectorJNI.releaseApriltagDetector(aprilTagPointer);
            aprilTagPointer = 0;
        } else {
            System.out.println("AprilTagDetectionPipeline.finalize(): aprilTagPointer was NULL");
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGBA2GRAY);

        synchronized (decimationSync) {
            if (needToSetDecimation) {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(aprilTagPointer, decimation);
                needToSetDecimation = false;
            }
        }

        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(aprilTagPointer, gray, tagSize, fx, fy, cx, cy);

        synchronized (detectionsUpdateSync) {
            detectionsUpdate = detections;
        }

        for (AprilTagDetection detection : detections) {
            Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagSize, tagSize);
            drawAxisMarker(input, tagSize / 2.0, 6, pose.rVector, pose.tVector, cameraMatrix);
            draw3DCubeMarker(input, tagSize, tagSize, tagSize, 5, pose.rVector, pose.tVector, cameraMatrix);
        }

        return input;
    }

    public void setDecimation(float decimation) {
        synchronized (decimationSync) {
            this.decimation = decimation;
            needToSetDecimation = true;
        }
    }

    public ArrayList<AprilTagDetection> getLatestDetections() {
        return detections;
    }

    public ArrayList<AprilTagDetection> getDetectionsUpdate() {
        synchronized (detectionsUpdateSync) {
            ArrayList<AprilTagDetection> ret = detectionsUpdate;
            detectionsUpdate = null;
            return ret;
        }
    }

    public void constructMatrix() {
        cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);

        cameraMatrix.put(0, 0, fx);
        cameraMatrix.put(0, 1, 0);
        cameraMatrix.put(0, 2, cx);

        cameraMatrix.put(1, 0, 0);
        cameraMatrix.put(1, 1, fy);
        cameraMatrix.put(1, 2, cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2, 1, 0);
        cameraMatrix.put(2, 2, 1);
    }

    public void drawAxisMarker(Mat buffer, double length, int thickness, Mat rVector, Mat tVector, Mat cameraMatrix) {
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(0, 0, 0),
                new Point3(length, 0, 0),
                new Point3(0, length, 0),
                new Point3(0, 0, -length)
        );

        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rVector, tVector, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        Imgproc.line(buffer, projectedPoints[0], projectedPoints[1], red, thickness);
        Imgproc.line(buffer, projectedPoints[0], projectedPoints[2], green, thickness);
        Imgproc.line(buffer, projectedPoints[0], projectedPoints[3], blue, thickness);

        Imgproc.circle(buffer, projectedPoints[0], thickness, white, -1);
    }

    public void draw3DCubeMarker(Mat buffer, double length, double tagWidth, double tagHeight, int thickness, Mat rVector, Mat tVector, Mat cameraMatrix) {
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(-tagWidth / 2, tagHeight / 2, 0),
                new Point3(tagWidth / 2, tagHeight / 2, 0),
                new Point3(tagWidth / 2, -tagHeight / 2, 0),
                new Point3(-tagWidth / 2, -tagHeight / 2, 0),
                new Point3(-tagWidth / 2, tagHeight / 2, -length),
                new Point3(tagWidth / 2, tagHeight / 2, -length),
                new Point3(tagWidth / 2, -tagHeight / 2, -length),
                new Point3(-tagWidth / 2, -tagHeight / 2, -length)
        );

        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rVector, tVector, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        for (int i = 0; i < 4; i++) {
            Imgproc.line(buffer, projectedPoints[i], projectedPoints[i + 4], blue, thickness);
        }

        Imgproc.line(buffer, projectedPoints[4], projectedPoints[5], green, thickness);
        Imgproc.line(buffer, projectedPoints[5], projectedPoints[6], green, thickness);
        Imgproc.line(buffer, projectedPoints[6], projectedPoints[7], green, thickness);
        Imgproc.line(buffer, projectedPoints[4], projectedPoints[7], green, thickness);
    }

    public Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagSizeX, double tagSizeY) {
        MatOfPoint2f points2D = new MatOfPoint2f(points);
        Point3[] arrayPoints3D = new Point3[4];

        arrayPoints3D[0] = new Point3(-tagSizeX / 2, tagSizeY / 2, 0);
        arrayPoints3D[1] = new Point3(tagSizeX / 2, tagSizeY / 2, 0);
        arrayPoints3D[2] = new Point3(tagSizeX / 2, -tagSizeY / 2, 0);
        arrayPoints3D[3] = new Point3(-tagSizeX / 2, -tagSizeY / 2, 0);

        MatOfPoint3f points3D = new MatOfPoint3f(arrayPoints3D);
        Pose pose = new Pose();
        Calib3d.solvePnP(points3D, points2D, cameraMatrix, new MatOfDouble(), pose.rVector, pose.tVector, false);

        return pose;
    }

    public static class Pose {
        Mat rVector;
        Mat tVector;

        public Pose() {
            rVector = new Mat();
            tVector = new Mat();
        }

        public Pose(Mat rVector, Mat tVector) {
            this.rVector = rVector;
            this.tVector = tVector;
        }
    }
}