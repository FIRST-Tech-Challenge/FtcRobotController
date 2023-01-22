package org.firstinspires.ftc.team6220_PowerPlay;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class GrabberCameraPipeline extends OpenCvPipeline {
    public double xPosition = Constants.CAMERA_CENTER_X;
    public double yPosition = Constants.CAMERA_CENTER_Y;
    public boolean detected = false;

    //fields
    List<MatOfPoint> contours = new ArrayList<>();
    Mat hierarchy = new Mat();
    Mat circleThresh = new Mat();
    Mat mask = new Mat();
    Mat circles = new Mat();
    Point centerPoint = new Point(Constants.CAMERA_CENTER_X, Constants.CAMERA_CENTER_Y);

    @Override
    public Mat processFrame(Mat input) {
        //make the circle that will be used to crop the image
        Imgproc.circle(circleThresh, centerPoint, 200, new Scalar(255, 255, 255), -1, 8, 0);

        //crop the input image based on the circle
        input.copyTo(mask, circleThresh);

        //transform the RGB frame into a HSV frame
        Imgproc.cvtColor(input, mask, Imgproc.COLOR_RGB2HSV);

        //blur the HSV frame
        Imgproc.GaussianBlur(mask, mask, Constants.BLUR_SIZE, 0);

        //mask the blurred frame
        Core.inRange(mask, Constants.LOWER_BLACK, Constants.UPPER_BLACK, mask);

        //detect circles in frame
        Imgproc.HoughCircles(mask, circles, Imgproc.HOUGH_GRADIENT,
                /*Inverse ration of resolution*/1,
                /*Minimum distance between circle centers*/Constants.CIRCLE_DETECTOR_MIN_DIST,
                /*Upper threshold for the internal Canny edge detector*/ Constants.CIRCLE_DETECTOR_UPPER_CANNY_THRESHOLD,
                /*Threshold for center detection*/ Constants.CIRCLE_DETECTOR_CENTER_DETECT_THRESHOLD,
                /*Minimum radius of detected circles*/ Constants.CIRCLE_DETECTOR_MIN_RADIUS,
                /*Maximum radius of detected circles*/ Constants.CIRCLE_DETECTOR_MAX_RADIUS);

        //find largest circle if one exists, and set the detected boolean to true
        if (circles.empty() == false) {
            detected = true;
            double maxCircle = 0.0;
            int maxCircleId = 0;

            //loop through circles and find the one with the largest radius, then assign the id of that circle to a variable
            for (int circleId = 0; circleId < circles.cols(); circleId++) {
                double[] data = circles.get(0, circleId);
                double currentRadius = data[2];
                if (maxCircle < currentRadius) {
                    maxCircle = currentRadius;
                    maxCircleId = circleId;
                }
            }

            //find the data for the largest circle
            Point center = new Point(Math.round(circles.get(0, maxCircleId)[0]), Math.round(Math.round(circles.get(0, maxCircleId)[1])));
            int radius = (int) Math.round(Math.round(circles.get(0, maxCircleId)[2]));
            Imgproc.circle(mask, center, radius, new Scalar(0, 0, 255), 3, 8, 0);

            //set Xpos and Ypos to the x and y of the circle
            xPosition = center.x;
            yPosition = center.y;

            //i3f no circles are found set the detected boolean to false and the Xpos and Ypos to 0,0
        } else {
            xPosition = Constants.CAMERA_CENTER_X;
            yPosition = Constants.CAMERA_CENTER_Y;
            detected = false;
        }

        contours.clear();
        return mask;
    }
}
