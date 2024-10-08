package org.firstinspires.ftc.teamcode.learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * ColorDetectionPipeline
 *
 * This class extends OpenCvPipeline and prosses each camera frame to detect yellow, blue, and red objects.
 * It uses color fresholding in the HSV color space and contour detection to identify significant objects.
 * Additionally, it determines whitch color is most prominent in the frame based on the area occupied and
 * draws a bounding box around the detected color.
 */
public class ColorDetectionPipeline extends OpenCvPipeline {
    // Define color ranges in HSV (Hue, Saturation, Value)
    // HSV ranges: Hue (0-179), Saturation (0-255), Value (0-255)

    //Yellow color range
    private final Scalar lowerYellow = new Scalar(95, 30, 30);
    private final Scalar upperYellow = new Scalar(110, 255, 255);



    //Blue color range
    private final Scalar lowerBlue = new Scalar(0, 30, 30);
    private final Scalar upperBlue = new Scalar(15, 255, 255);

    private final Scalar lowerRed = new Scalar(110, 30, 30);
    private final Scalar upperRed = new Scalar(125, 255, 255);
    //Red color range (note: red wraps around the hue spectrum, so we use two ranges)
//    private final Scalar lowerRed1 = new Scalar(0, 50, 70);
//    private final Scalar upperRed1 = new Scalar(9, 255, 255);
//    private final Scalar lowerRed2 = new Scalar(160, 50, 70);
//    private final Scalar upperRed2 = new Scalar(179, 255, 255);

    /**
     * Datected color
     * <p>
     * This enum represents the possible colors that can be detected.
     */
    public enum DetectedColor {
        NONE,
        YELLOW,
        BLUE,
        RED
    }

    //Output variable indicating the most prominent detected color
    public DetectedColor detectedColor = DetectedColor.NONE;

    //Area threshold to avoid false positives (adjust based on expected object size)
    private final double areaThreshold = 0;

    //Bounding box variables
    private Rect boundingBox = new Rect();

    public LinearOpMode myOpMode;

    /**
     * processFrame
     * <p>
     * This method is called on every frame captured by the webcam.
     * It processes the frame to detect specified colors, determines whitch color is most prominent,
     * and draws a bounding box around the detected color.
     *
     * @param input The current frame captuered by the webcam.
     * @return The processed frame with annotations (bounding box) if a color is detected,
     */
    @Override
    public Mat processFrame(Mat input) {
        //Convert the imput image from BGR to HSV color space
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

        //Create binary masks for each color based on defined HSV ranges
        Mat maskBlue = new Mat();
        Core.inRange(hsv, lowerBlue, upperBlue, maskBlue);
        //return (hsv);
        //return maskBlue;


        Mat maskYellow = new Mat();
        Core.inRange(hsv, lowerYellow, upperYellow, maskYellow);

        Mat maskRed = new Mat();
        Core.inRange(hsv, lowerRed, upperRed, maskRed);

//        Mat maskRed1 = new Mat();
//        Core.inRange(hsv, lowerRed1, upperRed1, maskRed1);
//        Mat maskRed2 = new Mat();
//        Core.inRange(hsv, lowerRed2, upperRed2, maskRed2);
//        double areaRed = calculateArea(maskRed1) + calculateArea(maskRed2);
//        myOpMode.telemetry.addData("Red Area 1:",calculateArea(maskRed1));
//        myOpMode.telemetry.addData("Red Area 2:",calculateArea(maskRed2));
//        Mat maskRed = new Mat();
        //       Core.add(maskRed1, maskRed2, maskRed); //Combine both red masks


        //Calculate the area covered by each color
        double areaYellow = calculateArea(maskYellow);
        double areaBlue = calculateArea(maskBlue);
        double areaRed = calculateArea(maskRed);
        myOpMode.telemetry.addData("Yellow Area:", areaYellow);
        myOpMode.telemetry.addData("Blue Area:", areaBlue);
        myOpMode.telemetry.addData("Red Area:", areaRed);

        //Determine whitch color has the maximum area
        if (areaYellow > areaThreshold || areaBlue > areaThreshold || areaRed > areaThreshold) {
            if (areaYellow >= areaBlue && areaYellow >= areaRed) {
                detectedColor = DetectedColor.YELLOW;
            } else if (areaBlue >= areaYellow && areaBlue >= areaRed) {
                detectedColor = DetectedColor.BLUE;
            } else {
                detectedColor = DetectedColor.RED;
            }
        }
      /*     //Find contours for the detected color to draw bounding box
            Mat mask = getMaskForDetectedColor();
            java.util.List<MatOfPoint> contours = new java.util.ArrayList<>();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if (!contours.isEmpty()) { //Page 4
                //Find the largest contour
                MatOfPoint largestContour = contours.get(0);
                double maxArea = Imgproc.contourArea(largestContour);
                for (MatOfPoint contour : contours) {
                    double area = Imgproc.contourArea(contour);
                    if (area > maxArea) {
                        largestContour = contour;
                        maxArea = area;
                    }
                }

                //Get bounding rectangle of the largest contour
                boundingBox = Imgproc.boundingRect(largestContour);

                //Draw the bounding box on the input frame
                Scalar boxColor = getColorScalar(detectedColor);
                Imgproc.rectangle(input, boundingBox.tl(), boundingBox.br(), boxColor, 3);
                }
           }

         else {
            detectedColor = DetectedColor.NONE;
        }
*/

        //Release Mats to free memery (prevent memery leaks)
        hsv.release();
        maskYellow.release();
        maskBlue.release();
        maskRed.release();
        //     maskRed2.release();
        maskRed.release();

        //Return the annotated frame
        return input;
        //return maskBlue;

    }

    /**
     * calculate Area
     * <p>
     * This helper method calculates the total area of all contours in the provided mask.
     *
     * @param mask The binary mask image foe a specific color.
     * @return The total area of detected contours.
     */
    private double calculateArea(Mat mask) {
        //Find contours in the mask.


        return Math.round(Core.sumElems(mask).val[0] / 100) / 100;

 /*       java.util.List<MatOfPoint> contours = new java.util.ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        //Calculate the total area of all contoure.
        double totalArea = 0.0;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > areaThreshold) { //Consider only significant contours
                totalArea += area;
            }
        }  */
        //return totalArea;
    }

    /**
     * getMaskForDetectedColor
     * <p>
     * This helper method returns the mask corresponding to the currently detected color.
     *
     * @return The binary mask Mat for the detected color.
     */
 /*   private Mat getMaskForDetectedColor() {
        switch (detectedColor) {
            case YELLOW:
                Mat maskYellow = new Mat();
                Core.inRange(new Mat(), lowerYellow, upperYellow, maskYellow);
                return maskYellow;
            case BLUE:
                Mat maskBlue = new Mat();
                Core.inRange(new Mat(), lowerBlue, upperBlue, maskBlue);
                return maskBlue;
            case RED:
                Mat maskRed1 = new Mat();
                Core.inRange(new Mat(), lowerRed1, upperRed1, maskRed1);
                Mat maskRed2 = new Mat();
                Core.inRange(new Mat(), lowerRed2, upperRed2, maskRed2);
                Mat maskRed = new Mat();
                Core.add(maskRed1, maskRed2, maskRed);
                return maskRed;
            case NONE:
            default:
                return new Mat();
        }
    }
*/
    /**
     * getColorScalar
     * <p>
     * This helper method returns the Scalar color for drawing based on the detected color.
     *
     * @param color The detected color.
     * @return The Scalar representing the BGR color for drawing.
     */
    private Scalar getColorScalar(DetectedColor color) {
        switch (color) {
            case YELLOW:
                return new Scalar(0, 255, 255); //Yellow in BGR
            case BLUE:
                return new Scalar(255, 0, 0); //Blue in BGR
            case RED:
                return new Scalar(0, 0, 255); //Red in BGR
            case NONE:
            default:
                return new Scalar(0, 0, 0); //Black (no color)
        }
    }

}
