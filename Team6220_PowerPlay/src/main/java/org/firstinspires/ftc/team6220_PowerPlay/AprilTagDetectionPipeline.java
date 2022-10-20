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
    private long aprilTagPointer;
    private final Mat gray = new Mat();
    private final Object detectionsUpdateSync = new Object();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();
    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();

    Mat cameraMatrix;
    Scalar blue = new Scalar(7, 197, 235, 255);
    Scalar red = new Scalar(255, 0, 0, 255);
    Scalar green = new Scalar(0, 255, 0, 255);
    Scalar white = new Scalar(255, 255, 255, 255);

    double fx;
    double fy;
    double cx;
    double cy;

    double tagSize;
    double tagSizeX;
    double tagSizeY;

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    public AprilTagDetectionPipeline(double tagSize, double fx, double fy, double cx, double cy) {
        this.tagSize = tagSize;
        this.tagSizeX = tagSize;
        this.tagSizeY = tagSize;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;

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
            Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagSizeX, tagSizeY);
            drawAxisMarker(input, tagSizeY / 2.0, 6, pose.rVector, pose.tVector, cameraMatrix);
            draw3DCubeMarker(input, tagSizeX, tagSizeX, tagSizeY, 5, pose.rVector, pose.tVector, cameraMatrix);
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
        cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

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

    /**
     * Draw a 3D axis marker on a detection. (Similar to what Vuforia does)
     *
     * @param buffer the RGB buffer on which to draw the marker
     * @param length the length of each of the marker 'poles'
     * @param rVector the rotation vector of the detection
     * @param tVector the translation vector of the detection
     * @param cameraMatrix the camera matrix used when finding the detection
     */
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
                new Point3( -tagWidth / 2, tagHeight / 2, 0),
                new Point3( tagWidth / 2, tagHeight / 2, 0),
                new Point3( tagWidth / 2, -tagHeight / 2, 0),
                new Point3( -tagWidth / 2, -tagHeight / 2, 0),
                new Point3( -tagWidth / 2, tagHeight / 2, -length),
                new Point3( tagWidth / 2, tagHeight / 2, -length),
                new Point3( tagWidth / 2, -tagHeight / 2, -length),
                new Point3( -tagWidth / 2, -tagHeight / 2, -length)
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

    /**
     * Extracts 6DOF pose from a trapezoid, using a camera intrinsics matrix and the
     * original size of the tag.
     *
     * @param points the points which form the trapezoid
     * @param cameraMatrix the camera intrinsics matrix
     * @param tagSizeX the original width of the tag
     * @param tagSizeY the original height of the tag
     * @return the 6DOF pose of the camera relative to the tag
     */
    public Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagSizeX, double tagSizeY) {
        // The actual 2d points of the tag detected in the image
        MatOfPoint2f points2D = new MatOfPoint2f(points);

        // The 3d points of the tag in an 'ideal projection'
        Point3[] arrayPoints3D = new Point3[4];
        arrayPoints3D[0] = new Point3(-tagSizeX / 2, tagSizeY / 2, 0);
        arrayPoints3D[1] = new Point3(tagSizeX / 2, tagSizeY / 2, 0);
        arrayPoints3D[2] = new Point3(tagSizeX / 2, -tagSizeY / 2, 0);
        arrayPoints3D[3] = new Point3(-tagSizeX / 2, -tagSizeY / 2, 0);
        MatOfPoint3f points3D = new MatOfPoint3f(arrayPoints3D);

        // Using this information, actually solve for pose
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
