package org.firstinspires.ftc.teamcode.vision.simulator;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.lang.Math;
import java.util.ArrayList;
import java.util.Collections;
public class pipeline1Simulator extends OpenCvPipeline {
    private Telemetry telemetry;
    //We declare the mats ontop so we can reuse them later to avoid memory leaks
    private Mat matYCrCb = new Mat();
    private Mat matCbBottom = new Mat();
    private Mat matCbMiddle = new Mat();
    private Mat matCbTop = new Mat();
    private Mat matCbBottom1 = new Mat();
    private Mat matCbMiddle1 = new Mat();
    private Mat matCbTop1 = new Mat();
    private Mat matCbBottom2 = new Mat();
    private Mat matCbMiddle2 = new Mat();
    private Mat matCbTop2 = new Mat();
    private Mat topBlock = new Mat();
    private Mat middleBlock = new Mat();
    private Mat bottomBlock = new Mat();

    //Where the average CB value of the rectangles are stored
    private double topAverage = 0;
    private double middleAverage = 0;
    private double bottomAverage = 0;

    //The max difference allowed inside the rectangles
    private int threshold = 15;

    //The position related to the screen
    private double topRectWidthPercentage = 0.25;
    private double topRectHeightPercentage = 0.50;
    private double middleRectWidthPercentage = 0.50;
    private double middleRectHeightPercentage = 0.50;
    private double bottomRectWidthPercentage = 0.75;
    private double bottomRectHeightPercentage = 0.50;

    //The width and height of the rectangles in terms of pixels
    private int rectangleWidth = 10;
    private int rectangleHeight = 10;

    public pipeline1Simulator(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {

        /**
         *input which is in RGB is the frame the camera gives
         *We convert the input frame to the color space matYCrCb
         *Then we store this converted color space in the mat matYCrCb
         *For all the color spaces go to
         *https://docs.opencv.org/3.4/d8/d01/group__imgproc__color__conversions.html
         */
        Imgproc.cvtColor(input, matYCrCb, Imgproc.COLOR_RGB2YCrCb);
        telemetry.addData("Telemetry","Telemetry");
        telemetry.addData("topBoxAverage",topAverage);
        telemetry.addData("topBoxAverage",middleAverage);
        telemetry.addData("bottomBoxAverage",bottomAverage);
        // USE Z-SCORE
        telemetry.update();
        //The points needed for the rectangles are calculated here
        Scalar red = new Scalar(255,0,0);
        Scalar yellow = new Scalar(255,255,0);
        Rect topRect = new Rect(
                (int) (matYCrCb.width() * topRectWidthPercentage),
                (int) (matYCrCb.height() * topRectHeightPercentage),
                rectangleWidth,
                rectangleHeight
        );
        Rect middleRect = new Rect(
                (int) (matYCrCb.width() * middleRectWidthPercentage),
                (int) (matYCrCb.height() * middleRectHeightPercentage),
                rectangleWidth,
                rectangleHeight
        );
        Rect bottomRect = new Rect(
                (int) (matYCrCb.width() * bottomRectWidthPercentage),
                (int) (matYCrCb.height() * bottomRectHeightPercentage),
                rectangleWidth,
                rectangleHeight
        );

        //The rectangle is drawn into the mat
        switch (mostDifferent(topAverage,middleAverage,bottomAverage)) {
            case 1:
                drawRectOnToMat(input, topRect, yellow);
                drawRectOnToMat(input, middleRect, red);
                drawRectOnToMat(input, bottomRect, red);
                break;
            case 2:
                drawRectOnToMat(input, topRect, red);
                drawRectOnToMat(input, middleRect, yellow);
                drawRectOnToMat(input, bottomRect, red);
                break;
            case 3:
                drawRectOnToMat(input, topRect, red);
                drawRectOnToMat(input, middleRect, red);
                drawRectOnToMat(input, bottomRect, yellow);
                break;
        }

        //We crop the image so it is only everything inside the rectangles and find the cb value inside of them
        topBlock = matYCrCb.submat(topRect);
        middleBlock = matYCrCb.submat(middleRect);
        bottomBlock = matYCrCb.submat(bottomRect);
        Core.extractChannel(topBlock, matCbTop, 0);
        Core.extractChannel(middleBlock,matCbMiddle,0);
        Core.extractChannel(bottomBlock, matCbBottom, 0);
        Core.extractChannel(topBlock, matCbTop1, 1);
        Core.extractChannel(middleBlock,matCbMiddle1,1);
        Core.extractChannel(bottomBlock, matCbBottom1, 1);
        Core.extractChannel(topBlock, matCbTop2, 2);
        Core.extractChannel(middleBlock,matCbMiddle2,2);
        Core.extractChannel(bottomBlock, matCbBottom2, 2);

        Scalar topMean = Core.mean(matCbTop);
        Scalar topMean1 = Core.mean(matCbTop1);
        Scalar topMean2 = Core.mean(matCbTop2);
        Scalar middleMean = Core.mean(matCbMiddle);
        Scalar middleMean1 = Core.mean(matCbMiddle1);
        Scalar middleMean2 = Core.mean(matCbMiddle2);
        Scalar bottomMean = Core.mean(matCbBottom);
        Scalar bottomMean1 = Core.mean(matCbBottom1);
        Scalar bottomMean2 = Core.mean(matCbBottom2);

        topAverage = topMean.val[0] + topMean1.val[0] + topMean2.val[0];
        middleAverage = middleMean.val[0] + middleMean1.val[0] + middleMean2.val[0];
        bottomAverage = bottomMean.val[0] + bottomMean1.val[0] + bottomMean2.val[0];


        //return the mat to be shown onto the screen
        return input;
    }

    private int mostDifferent(double box1, double box2, double box3) {
        double boxMean = (box1+box2+box3)/3;
        double diffBox1 = Math.abs(boxMean - box1);
        double diffBox2 = Math.abs(boxMean - box2);
        double diffBox3 = Math.abs(boxMean - box3);
        ArrayList<Double> list = new ArrayList<>();
        list.add(diffBox1);
        list.add(diffBox2);
        list.add(diffBox3);
        double boxNum = Collections.max(list);
        return list.indexOf(boxNum) + 1;
    }

    /**
     * Draw the rectangle onto the desired mat
     *
     * @param mat   The mat that the rectangle should be drawn on
     * @param rect  The rectangle
     * @param color The color the rectangle will be
     */
    private void drawRectOnToMat(Mat mat, Rect rect, Scalar color) {
        Imgproc.rectangle(mat, rect, color, 1);
    }

    public double getTopAverage() {
        return topAverage;
    }

    public double getMiddleAverage() { return middleAverage; }

    public double getBottomAverage() {
        return bottomAverage;
    }

    public void setThreshold(int threshold) {
        this.threshold = threshold;
    }

    public int getThreshold() {
        return threshold;
    }

    public void setTopRectWidthPercentage(double topRectWidthPercentage) {
        this.topRectWidthPercentage = topRectWidthPercentage;
    }

    public void setTopRectHeightPercentage(double topRectHeightPercentage) {
        this.topRectHeightPercentage = topRectHeightPercentage;
    }

    public void setBottomRectWidthPercentage(double bottomRectWidthPercentage) {
        this.bottomRectWidthPercentage = bottomRectWidthPercentage;
    }

    public void setBottomRectHeightPercentage(double bottomRectHeightPercentage) {
        this.bottomRectHeightPercentage = bottomRectHeightPercentage;
    }

    public void setRectangleWidth(int rectangleWidth) {
        this.rectangleWidth = rectangleWidth;
    }

    public void setRectangleHeight(int rectangleHeight) {
        this.rectangleHeight = rectangleHeight;
    }
}
