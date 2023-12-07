package org.firstinspires.ftc.teamcode.Autos;

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

//most of this class is just setup that doesn't need to be changed
//the core.inRange line is what controls the important stuff
public class RedFinder extends OpenCvPipeline
{
    // Coordinate position of the top left corner of the selected rectangle
    public static Point screenPosition = new Point(0,0);

    private Mat
            rawImage,       // Raw image output from the camera
            workingMat,     // The image currently being worked on and being modified
            selectionMask,
            hierarchy;

    /**
     * Sets up all the variables to keep code clean
     */
    public RedFinder() {
        rawImage = new Mat();
        workingMat = new Mat();
        selectionMask = new Mat();
        hierarchy = new Mat();
    }


    @Override
    public Mat processFrame(Mat input) {
        // Copies the original input to other materials to be worked on so they aren't overriding each other
        input.copyTo(rawImage);
        input.copyTo(workingMat);
        input.copyTo(selectionMask);


        // Sets the best fitting rectangle to the one currently selected
        Rect bestRect = new Rect();

        // Numerical value for the "best fit" rectangle
        // MAX_VALUE to find the lesser difference
        double lowestScore = Double.MAX_VALUE;

        //converts the image from rgb to hsv
        Imgproc.cvtColor(rawImage,workingMat,Imgproc.COLOR_RGB2HSV);

        //controls the color range the camera is looking for in the hsv color space
        //the hue value is scaled by .5, the saturation and value are scaled by 2.55
        Core.inRange(workingMat,new Scalar(0,60,60),new Scalar(15,255,255),workingMat);

        // Creates a list for all contoured objects the camera will find
        List<MatOfPoint> contoursList = new ArrayList<>();

        Imgproc.findContours(workingMat, contoursList, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(selectionMask, contoursList,-1, new Scalar(40,40,40),2);

        // Scores all the contours and selects the best of them
        for(MatOfPoint contour : contoursList){
            // Calculate the "score" of the selected contour
            double score = calculateScore(contour);

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(contour);

            // Draw the current found rectangle on the selections mask
            //     Drawn in blue
            Imgproc.rectangle(selectionMask, rect.tl(), rect.br(), new Scalar(0,0,255),2);

            // If the result is better then the previously tracked one,
            // and the top left coordinates are within the cropped area
            // set this rect as the new best
            if(score < lowestScore){
                lowestScore = score;
                bestRect = rect;
            }
        }

        // Draw the "best fit" rectangle on the selections mask and skystone only mask
        //     Drawn in red
        Imgproc.rectangle(selectionMask, bestRect.tl(), bestRect.br(), new Scalar(0,255,0),10);

        // Sets the position of the selected rectangle (relative to the screen resolution)
        screenPosition = new Point(bestRect.x, bestRect.y);

        return selectionMask;

    }


    private double calculateScore(Mat input) {
        // Validates input, returning the maximum value if invalid
        if(!(input instanceof MatOfPoint))
            return Double.MAX_VALUE;
        // Otherwise returns the calculated area of the contour
        return -Imgproc.contourArea(input);
    }
}