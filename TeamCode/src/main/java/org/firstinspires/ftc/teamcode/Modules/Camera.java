package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

//opencv for vision stuff
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

/*
 * This is the pipeline which will be processing the cameras feed
 * It should be able to:
 *  - detect and recognize april tags
 *  - detect and locate (if I will we able to) artifacts
 *  - get the current state of the ramp
 *
 * welp lets see how well I make this
 */

// omg this is going to need so much cleaning up
public class Camera extends OpenCvPipeline {

    ///////////////////////////////////////////////
    ////                                     /////
    ////              VARIABLES              /////
    ////                                     /////
    //////////////////////////////////////////////

    // -------------- ARTIFACTS -------------- //

    /*
    Purple lower: (0, 136.0, 120.4)
    Purple upper: (255, 255, 255)
     */
    public Scalar lower = new Scalar(0, 136.0, 120.4);
    public Scalar upper = new Scalar(255, 255, 255);


    private final Mat ycrcbmat = new Mat();
    private final Mat binaryMat = new Mat();
    private final Mat maskedInputMat = new Mat();
    private final Mat grayscaleMat = new Mat();

    // Contour Vars

    List<MatOfPoint> contours = new ArrayList<>();
    public double lowerContourThreshold = 0;
    public double upperContourThreshold = 300;
    public Scalar contourColors = new Scalar(0,255,0);
    public double noiseSensitivity = 153;
    public int contourSize = 1;

    private final Mat edgeDetectorFrame = new Mat();
    private int onlyContours = 1;

    public static double currentLargest = 0;

    Mat activeMat;
    Mat emptyMat = Mat.zeros(edgeDetectorFrame.size(), CvType.CV_8UC3);
    int index = 0;
    double contourArea;
    double largestArea = 0;

    // -------------- APRIL TAGS ------------- //

    private long nativeApriltagPtr;
    private Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();


    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    Mat cameraMatrix;

    Scalar blue = new Scalar(7,197,235,255);
    Scalar red = new Scalar(255,0,0,255);
    Scalar green = new Scalar(0,255,0,255);
    Scalar white = new Scalar(255,255,255,255);

    double fx;
    double fy;
    double cx;
    double cy;

    // UNITS ARE METERS
    double tagsize;
    double tagsizeX;
    double tagsizeY;

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    ///////////////////////////////////////////////
    ////                                     /////
    ////              FUNCTIONS              /////
    ////                                     /////
    //////////////////////////////////////////////

    public Camera(double tagsize, double fx, double fy, double cx, double cy) {
        this.tagsize = tagsize;
        this.tagsizeX = tagsize;
        this.tagsizeY = tagsize;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;

        constructMatrix();

        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);

    }

    @Override
    public Mat processFrame(Mat input) {

        // Convert to greyscale
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

        synchronized (decimationSync)
        {
            if(needToSetDecimation)
            {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                needToSetDecimation = false;
            }
        }

        // Run AprilTag
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);

        synchronized (detectionsUpdateSync)
        {
            detectionsUpdate = detections;
        }

        // For fun, use OpenCV to draw 6DOF markers on the image. We actually recompute the pose using
        // OpenCV because I haven't yet figured out how to re-use AprilTag's pose in OpenCV.
        for(AprilTagDetection detection : detections)
        {
            Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagsizeX, tagsizeY);
            drawAxisMarker(input, tagsizeY/2.0, 6, pose.rvec, pose.tvec, cameraMatrix);
            draw3dCubeMarker(input, tagsizeX, tagsizeX, tagsizeY, 5, pose.rvec, pose.tvec, cameraMatrix);
        }

        //Takes our "input" mat as an input, and outputs to a separate Mat buffer "ycrcbMat"
        Imgproc.cvtColor(input, ycrcbmat, Imgproc.COLOR_RGB2YCrCb);
        //Order is source, lowerBound, upperbound, dst.
        Core.inRange(ycrcbmat, lower, upper, binaryMat);
        /*
         * Release the reusable Mat so that old data doesn't
         * affect the next step in the current processing
         */
        maskedInputMat.release();

        //Order: src1, src2, dst, mask.
        Core.bitwise_and(input, input, maskedInputMat, binaryMat);

        // now the masked input mat is the filtered image with colors and shit.
        // Sorry Alok I ripped u off ;P

        // filter maskedinputmat to grey.
        Imgproc.cvtColor(maskedInputMat, grayscaleMat, Imgproc.COLOR_RGB2GRAY);

        // Order: input image, output edges(Array), lowerthreshold, upperthreshold
        Imgproc.Canny(grayscaleMat, edgeDetectorFrame, lowerContourThreshold, upperContourThreshold);

        contours.clear();
        //Order : input image, the contours from output, hierarchy, mode, and method

        Imgproc.findContours(edgeDetectorFrame, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];

        Rect[] boundRect = new Rect[contours.size()];
        //Might be useful later, idk so I am going to leave this here
        //Point[] centers = new Point[contours.size()];


        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(contours.get(i));
            //centers[i] = new Point();
        }


        List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
        for (MatOfPoint2f poly : contoursPoly) {
            //Just to make sure that no problems really come up.
            if(poly == null)
            {
                break;
            }

            contoursPolyList.add(new MatOfPoint(poly.toArray()));
        }


        // now this can be removed during the meet.
        if (onlyContours == 1)
        {
            activeMat = maskedInputMat;
        }
        else if (onlyContours == 2)
        {
            activeMat = emptyMat;
        }
        else {
            activeMat = input;
        }

        largestArea = 0;

        for (int i = 0; i < contours.size(); i++) {

            MatOfPoint contour = contours.get(i);

            contourArea = Imgproc.contourArea(contour);
            // A simple filter I cam up with ;P
            if (contourArea < noiseSensitivity)
            {
                continue;
            }

            if (contourArea > largestArea)
            {
                largestArea = contourArea;
                index = i;
            }

        }

        currentLargest = largestArea;


        // pretty sure that we actually don't need this for the meet either,
        // but is good for demonstration purposes and debugging
        if (contours.size() != 0 && largestArea != 0)
        {
            // Adding Text3
            Imgproc.putText (
                    activeMat,                                                             // Matrix obj of the image
                    largestArea + "",                                                      // Text to be added
                    new Point(boundRect[index].tl().x, boundRect[index].tl().y),           // point
                    Imgproc.FONT_HERSHEY_SIMPLEX ,                                         // front face
                    1,                                                                     // front scale
                    contourColors,                                                         // Scalar object for color
                    2                                                                      // Thickness
            );
            Imgproc.drawContours(activeMat, contoursPolyList, index, contourColors, contourSize);
            Imgproc.rectangle(activeMat, boundRect[index].tl(), boundRect[index].br(), contourColors, 2);
        }

        return activeMat;
    }

    // I think we could have just returned a Mat frame and have the FSM and Auton do the computing, but that is like
    // not smart
    public static double getLargestSize()
    {
        return currentLargest;
    }

    // boilerplate code if there is a better way to do this, please tell me
    @Override
    public void onViewportTapped()
    {
        if (onlyContours == 1 || onlyContours == 2)
            onlyContours++;
        else
            onlyContours = 1;
    }

    @Override
    public void finalize()
    {
        // Might be null if createApriltagDetector() threw an exception
        if(nativeApriltagPtr != 0)
        {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
            nativeApriltagPtr = 0;
        }
        else
        {
            System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
        }
    }

    public void setDecimation(float decimation)
    {
        synchronized (decimationSync)
        {
            this.decimation = decimation;
            needToSetDecimation = true;
        }
    }

    public ArrayList<AprilTagDetection> getLatestDetections()
    {
        return detections;
    }

    public ArrayList<AprilTagDetection> getDetectionsUpdate()
    {
        synchronized (detectionsUpdateSync)
        {
            ArrayList<AprilTagDetection> ret = detectionsUpdate;
            detectionsUpdate = null;
            return ret;
        }
    }

    void constructMatrix()
    {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //

        cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

        cameraMatrix.put(0,0, fx);
        cameraMatrix.put(0,1,0);
        cameraMatrix.put(0,2, cx);

        cameraMatrix.put(1,0,0);
        cameraMatrix.put(1,1,fy);
        cameraMatrix.put(1,2,cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2,1,0);
        cameraMatrix.put(2,2,1);
    }

    /**
     * Draw a 3D axis marker on a detection. (Similar to what Vuforia does)
     *
     * @param buf the RGB buffer on which to draw the marker
     * @param length the length of each of the marker 'poles'
     * @param rvec the rotation vector of the detection
     * @param tvec the translation vector of the detection
     * @param cameraMatrix the camera matrix used when finding the detection
     */
    void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    {
        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(0,0,0),
                new Point3(length,0,0),
                new Point3(0,length,0),
                new Point3(0,0,-length)
        );

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Draw the marker!
        Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, thickness);

        Imgproc.circle(buf, projectedPoints[0], thickness, white, -1);
    }

    void draw3dCubeMarker(Mat buf, double length, double tagWidth, double tagHeight, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    {
        //axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
        //       [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])

        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(-tagWidth/2, tagHeight/2,0),
                new Point3( tagWidth/2, tagHeight/2,0),
                new Point3( tagWidth/2,-tagHeight/2,0),
                new Point3(-tagWidth/2,-tagHeight/2,0),
                new Point3(-tagWidth/2, tagHeight/2,-length),
                new Point3( tagWidth/2, tagHeight/2,-length),
                new Point3( tagWidth/2,-tagHeight/2,-length),
                new Point3(-tagWidth/2,-tagHeight/2,-length));

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Pillars
        for(int i = 0; i < 4; i++)
        {
            Imgproc.line(buf, projectedPoints[i], projectedPoints[i+4], blue, thickness);
        }

        // Base lines
        //Imgproc.line(buf, projectedPoints[0], projectedPoints[1], blue, thickness);
        //Imgproc.line(buf, projectedPoints[1], projectedPoints[2], blue, thickness);
        //Imgproc.line(buf, projectedPoints[2], projectedPoints[3], blue, thickness);
        //Imgproc.line(buf, projectedPoints[3], projectedPoints[0], blue, thickness);

        // Top lines
        Imgproc.line(buf, projectedPoints[4], projectedPoints[5], green, thickness);
        Imgproc.line(buf, projectedPoints[5], projectedPoints[6], green, thickness);
        Imgproc.line(buf, projectedPoints[6], projectedPoints[7], green, thickness);
        Imgproc.line(buf, projectedPoints[4], projectedPoints[7], green, thickness);
    }

    /**
     * Extracts 6DOF pose from a trapezoid, using a camera intrinsics matrix and the
     * original size of the tag.
     *
     * @param points the points which form the trapezoid
     * @param cameraMatrix the camera intrinsics matrix
     * @param tagsizeX the original width of the tag
     * @param tagsizeY the original height of the tag
     * @return the 6DOF pose of the camera relative to the tag
     */
    Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagsizeX , double tagsizeY)
    {
        // The actual 2d points of the tag detected in the image
        MatOfPoint2f points2d = new MatOfPoint2f(points);

        // The 3d points of the tag in an 'ideal projection'
        Point3[] arrayPoints3d = new Point3[4];
        arrayPoints3d[0] = new Point3(-tagsizeX/2, tagsizeY/2, 0);
        arrayPoints3d[1] = new Point3(tagsizeX/2, tagsizeY/2, 0);
        arrayPoints3d[2] = new Point3(tagsizeX/2, -tagsizeY/2, 0);
        arrayPoints3d[3] = new Point3(-tagsizeX/2, -tagsizeY/2, 0);
        MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

        // Using this information, actually solve for pose
        Pose pose = new Pose();
        Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);

        return pose;
    }

    /*
     * A simple container to hold both rotation and translation
     * vectors, which together form a 6DOF pose.
     */
    class Pose
    {
        Mat rvec;
        Mat tvec;

        public Pose()
        {
            rvec = new Mat();
            tvec = new Mat();
        }

        public Pose(Mat rvec, Mat tvec)
        {
            this.rvec = rvec;
            this.tvec = tvec;
        }
    }

}
