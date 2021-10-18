package org.firstinspires.ftc.teamcode.vision.simulator;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.lang.Math;
public class pipeline1Simulator extends OpenCvPipeline {
    private final Telemetry telemetry;
    private final Scalar red = new Scalar(255,0,0);
    private final Scalar yellow = new Scalar(255,255,0);
    private final Mat matYCrCb = new Mat();
    private final Mat matCbBottom = new Mat();
    private final Mat matCbMiddle = new Mat();
    private final Mat matCbTop = new Mat();
    private final Mat matCbBottom1 = new Mat();
    private final Mat matCbMiddle1 = new Mat();
    private final Mat matCbTop1 = new Mat();
    private final Mat matCbBottom2 = new Mat();
    private final Mat matCbMiddle2 = new Mat();
    private final Mat matCbTop2 = new Mat();
    private Mat topBlock = new Mat();
    private Mat middleBlock = new Mat();
    private Mat bottomBlock = new Mat();

    //Where the average CB value of the rectangles are stored
    private double topAverage = 0;
    private double middleAverage = 0;
    private double bottomAverage = 0;

    //The position related to the screen
    private double topRectWidthPercentage = 0.25;
    private double topRectHeightPercentage = 0.50;
    private double middleRectWidthPercentage = 0.50;
    private double middleRectHeightPercentage = 0.50;
    private double bottomRectWidthPercentage = 0.75;
    private double bottomRectHeightPercentage = 0.50;

    //The width and height of the rectangles in terms of pixels
    private final int rectangleWidth = 10;
    private final int rectangleHeight = 10;


    public pipeline1Simulator(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * @param input input frame matrix
     */
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, matYCrCb, Imgproc.COLOR_RGB2YCrCb);
        telemetry.addData("topBoxAverage",topAverage);
        telemetry.addData("topBoxAverage",middleAverage);
        telemetry.addData("bottomBoxAverage",bottomAverage);
        telemetry.update();

        //The points needed for the rectangles are calculated here
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

        topAverage = topMean.val[0] + 0.5 * (topMean1.val[0] + topMean2.val[0]);
        middleAverage = middleMean.val[0] + 0.5 * (middleMean1.val[0] + middleMean2.val[0]);
        bottomAverage = bottomMean.val[0] + 0.5 * (bottomMean1.val[0] + bottomMean2.val[0]);


        //return the mat to be shown onto the screen
        return input;
    }

    public static int mostDifferent(double box1, double box2, double box3) {
        double boxMean = (box1+box2+box3)/3;
        double array[] = {Math.abs(boxMean - box1),Math.abs(boxMean - box2),Math.abs(boxMean - box3)};
        int max = array[0] > array[1] ? 1 : 2;
        return array[2] > array[max-1] ? 3 : max;
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

    /**
     * percentages of all rectangles. it goes top width, top height, middle width, etc.
     */
    public void setRectangles(double topRectWidthPercentage, double topRectHeightPercentage,double middleRectWidthPercentage,double middleRectHeightPercentage,double bottomRectWidthPercentage,double bottomRectHeightPercentage) {
        this.topRectWidthPercentage = topRectWidthPercentage;
        this.topRectHeightPercentage = topRectHeightPercentage;
        this.middleRectWidthPercentage = middleRectWidthPercentage;
        this.middleRectHeightPercentage = middleRectHeightPercentage;
        this.bottomRectWidthPercentage = bottomRectWidthPercentage;
        this.bottomRectHeightPercentage = bottomRectHeightPercentage;
    }
}
