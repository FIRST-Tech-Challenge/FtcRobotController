package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SignalPipeline extends OpenCvPipeline {
    private int width = 640;
    private int height = 480;
    private int hueSearchSize = 25;
    private int cornerSearchSize = 100;
    
    private Mat empty = new Mat();
    private Mat hsv = new Mat();
    private Mat gray = new Mat();
    private Mat graySearch;
    private Mat canny = new Mat();
    private Mat hue = new Mat();
    private Mat display = new Mat();
    private Mat contourHierarchy = new Mat();
    private Mat hueSearch;
    private Mat cornerSearch;

    // Points to determine what area we are looking in
    Point center = new Point(width / 2.0, height / 2.0);
    Point hueTopLeft = new Point(center.x - hueSearchSize / 2.0, center.y - hueSearchSize / 2.0);
    Point hueBottomRight = new Point(center.x + hueSearchSize / 2.0, center.y + hueSearchSize / 2.0);
    Point cornerTopLeft = new Point(center.x - cornerSearchSize / 2.0, center.y - cornerSearchSize / 2.0);
    Point cornerBottomRight = new Point(center.x + cornerSearchSize / 2.0, center.y + cornerSearchSize / 2.0);
    
    public volatile double meanHue = 0;
    public volatile int foundCorners = 0;
    public volatile int stateByHue = 0;
    public volatile int stateByCorners = 0;
    
    private void updateHsv(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(hsv, hue, 0);
        hueSearch = hue.submat(new Rect(hueTopLeft, hueBottomRight));
        
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
        cornerSearch = gray.submat(new Rect(cornerTopLeft, cornerBottomRight));
        graySearch = gray.submat(new Rect(cornerTopLeft, cornerBottomRight));
    }
    
    private void determineState() {
        // Difference from mean hue to the color of each state
        double hueDiff1 = Math.abs(meanHue - 7);
        double hueDiff2 = Math.abs(meanHue - 100);
        double hueDiff3 = Math.abs(meanHue - 82);
        
        // Difference from the corners found to the number of corners of each state
        double cornerDiff1 = Math.abs(foundCorners - 3);
        double cornerDiff2 = Math.abs(foundCorners - 5);
        double cornerDiff3 = Math.abs(foundCorners - 20);
        
        // Find the smallest difference from each
        double smallestHueDiff = Math.min(hueDiff1, Math.min(hueDiff2, hueDiff3));
        double smallestCornerDiff = Math.min(cornerDiff1, Math.min(cornerDiff2, cornerDiff3));
        
        // Find the matching difference
        if (hueDiff1 == smallestHueDiff) stateByHue = 1;
        else if (hueDiff2 == smallestHueDiff) stateByHue = 2;
        else if (hueDiff3 == smallestHueDiff) stateByHue = 3;

        if (cornerDiff1 == smallestCornerDiff) stateByCorners = 1;
        else if (cornerDiff2 == smallestCornerDiff) stateByCorners = 2;
        else if (cornerDiff3 == smallestCornerDiff) stateByCorners = 3;
    }

    @Override
    public void init(Mat mat) {
        updateHsv(mat);
    }
    
    @Override
    public Mat processFrame(Mat input) {
        updateHsv(input);
        meanHue = Core.mean(hueSearch).val[0];
        
        int maxCorners = 0;
        MatOfPoint corners = new MatOfPoint();
        double qualityLevel = 0.02;
        double minDistance = 12;
        int blockSize = 4, gradientSize = 3;
        boolean useHarris = false;
        double k = 0.04;
        
        Imgproc.goodFeaturesToTrack(cornerSearch, corners, maxCorners, qualityLevel, minDistance, empty, blockSize, gradientSize, useHarris, k);
        foundCorners = corners.rows();
        
        determineState();
        
        Imgproc.Canny(graySearch, canny, 100, 200);
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(canny, contours, contourHierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_TC89_KCOS);
        
        Imgproc.cvtColor(gray, display, Imgproc.COLOR_GRAY2RGB);
        // Draw an rectangle to show where we are sampling
        Imgproc.rectangle(display, hueTopLeft, hueBottomRight, new Scalar(255, 0, 0));
        Imgproc.rectangle(display, new Rect(cornerTopLeft, cornerBottomRight), new Scalar(0, 255, 0));
        
        // Draw the corners that were detected
        int[] cornersData = new int[(int) (corners.total() * corners.channels())];
        corners.get(0, 0, cornersData);
        for (int i = 0; i < corners.rows(); i++) {
            //Imgproc.circle(display, new Point(cornersData[i * 2] + cornerTopLeft.x, cornersData[i * 2 + 1] + cornerTopLeft.y), 4, new Scalar(0, 0, 255));
        }
        
        if (contours.size() > 0) {
            Imgproc.putText(display, String.valueOf(contours.get(0).rows()), new Point(100, 100), 0, 1, new Scalar(255, 255, 255));
        }
            
        Imgproc.drawContours(display, contours, -1, new Scalar(255, 255));
        
        // Matrices don't automatically free their memory
        corners.release();
        
        return display;
    }
}
