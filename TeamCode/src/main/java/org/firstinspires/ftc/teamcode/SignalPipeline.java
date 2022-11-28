package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SignalPipeline extends OpenCvPipeline {
    private int width = 640;
    private int height = 480;
    private int searchWidth = 100;
    
    private Mat hsv = new Mat();
    private Mat hsvSub = new Mat();
    private Mat gray = new Mat();
    private Mat graySub = new Mat();
    private Mat canny = new Mat();
    private Mat display = new Mat();
    private Mat contourHierarchy = new Mat();
    private Mat contourMask = new Mat();

    // Points to determine what area we are searching in
    Point center = new Point(width / 2.0, height / 2.0);
    Point searchTopLeft = new Point(center.x - searchWidth / 2.0, center.y - searchWidth / 2.0);
    Point searchBottomRight = new Point(center.x + searchWidth / 2.0, center.y + searchWidth / 2.0);
    
    private void updateConvertedMats(Mat input) {
        // Create sub matrices for our search zone
        hsvSub = input.submat(new Rect(searchTopLeft, searchBottomRight));
        graySub = input.submat(new Rect(searchTopLeft, searchBottomRight));
        
        Imgproc.cvtColor(hsvSub, hsv, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(graySub, gray, Imgproc.COLOR_RGB2GRAY);
    }
    
    private void determineState() {
        
    }

    @Override
    public void init(Mat mat) {
        // This needs to be called on initialization to set the matrices
        updateConvertedMats(mat);
    }
    
    @Override
    public Mat processFrame(Mat input) {
        updateConvertedMats(input);

        // Determine contours of the image using canny edge detection
        Imgproc.Canny(gray, canny, 100, 200);
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(canny, contours, contourHierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_TC89_KCOS);

        // TODO: This might not work if it detects more than one contour
        // Approximate the contour, because we will never have perfect
        MatOfPoint2f contourPoints = new MatOfPoint2f();
        MatOfPoint2f contourApprox = new MatOfPoint2f();

        if (contours.size() > 0) {
            contourPoints = new MatOfPoint2f(contours.get(0));
            double epsilon = 0.1 * Imgproc.arcLength(new MatOfPoint2f(contours.get(0)), true);
            Imgproc.approxPolyDP(contourPoints, contourApprox, epsilon, true);
        }

        // Create a mask with the contours
        Imgproc.drawContours(contourMask, contours, 0, new Scalar(255, 255, 255), Core.FILLED);
        // Find the mean hue using the contour mask
        double meanHue = Core.mean(hsv, contourMask).val[0];

        determineState();

        // Display what we are looking at, and any debug info
        display = input;
        //Imgproc.cvtColor(input, display, Imgproc.COLOR_GRAY2RGB);
        Imgproc.rectangle(display, new Rect(searchTopLeft, searchBottomRight), new Scalar(0, 255, 0));

        Imgproc.drawContours(display, Arrays.asList(new MatOfPoint(contourApprox)), -1, new Scalar(255, 255));
        Imgproc.putText(display, String.valueOf(meanHue), new Point(100, 100), 0, 2, new Scalar(255));

        return display;
    }
}
