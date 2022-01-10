package org.firstinspires.ftc.teamcode.auto.cv;

import static org.firstinspires.ftc.teamcode.common.Direction.IN_FRONT;
import static org.firstinspires.ftc.teamcode.common.Direction.LEFT;
import static org.firstinspires.ftc.teamcode.common.Direction.NOT_FOUND;
import static org.firstinspires.ftc.teamcode.common.Direction.NOT_INITIALIZED;
import static org.firstinspires.ftc.teamcode.common.Direction.RIGHT;
import static org.firstinspires.ftc.teamcode.common.utils.FTCConstants.FRAME_WIDTH;
import static org.firstinspires.ftc.teamcode.common.utils.FTCConstants.LEFT_ROI;
import static org.firstinspires.ftc.teamcode.common.utils.FTCConstants.MIDDLE_ROI;
import static org.firstinspires.ftc.teamcode.common.utils.FTCConstants.PERCENT_COLOR_THRESHOLD;
import static org.firstinspires.ftc.teamcode.common.utils.FTCConstants.RIGHT_ROI;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Direction;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * This is the pipeline for the AutoCVII class, pipelines render the photo and send logic from opencv
 * @author aryansinha
 */
public class CvPipeline extends OpenCvPipeline {
    //Create a variable for telemetry we can use
    private final Telemetry telemetry;
    //The distance from camera to the object.
    private volatile double distance;
    //Init Location
    private volatile Direction direction;
    //Whether using duck as shipping element.
    private boolean useDuck = true;
    //THIS IS A CHANGE

    private boolean left = false; // true if regular stone found on the left side
    private boolean right = false; // "" "" on the right side

    /**
     * Creates an instance of this class.
     * @param telemetry The telemetry object.
     */
    public CvPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.direction = NOT_INITIALIZED;
    }

    /**
     * Looks throuh the frame
     * Turns it into a matrix called input
     * @param input the input frame
     * @return a matrix that becomes a processed frame
     * @author aryansinha
     */
    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        //convert camera feed to HSV for easy implementations
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2HSV);

        // Low and High HSV ranges for Orange color (23-32).
        // Caution: Don't change the following values without consulting the coach.
        Scalar lowHSV = useDuck ? new Scalar( 25, 25, 35 ) : new Scalar(5, 50, 50);
        Scalar highHSV = useDuck ? new Scalar( 40, 255, 255 ) : new Scalar(40, 255, 255);

        //Threshold the incoming image to the selected HSV ranges.
        Mat thresh = new Mat();
        Core.inRange(mat, lowHSV, highHSV, thresh);

        //use rectangles as references
        Mat left = thresh.submat(LEFT_ROI);
        Mat right = thresh.submat(RIGHT_ROI);
        Mat middle = thresh.submat(MIDDLE_ROI);

        //Amount of areas on the left and right
        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;

        //Release the memory to avoid leaks.
        left.release();
        right.release();
        middle.release();
        mat.release();

        //Doing additional debug logging.
        logDebugData(leftValue, middleValue, rightValue, left, right, middle);

        //Set the direction to move to.
        setDirection(leftValue, middleValue, rightValue);

        Imgproc.cvtColor(thresh, thresh, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar(255, 0, 0);
        Scalar colorSkystone = new Scalar(0, 255, 0);
        Imgproc.rectangle(thresh, LEFT_ROI, direction == LEFT ? colorSkystone:colorStone);
        Imgproc.rectangle(thresh, RIGHT_ROI, direction == RIGHT ? colorSkystone:colorStone);
        Imgproc.rectangle(thresh, MIDDLE_ROI, direction == IN_FRONT ? colorSkystone:colorStone);

        return thresh;
    }

    /**
     * Gets the location
     * @return the location
     * @author aryansinha
     */
    public Direction getLocation() {
        return direction;
    }

    /**
     * Returns a valid distance.
     * @return A valid distance.
     */
    public double getDistance() {
        return distance;
    }

    /**
     * Code for finding the distance
     * This code uses the formula F = (P x D) / W
     * With some algebra that becomes D = (F x W) / P
     * The logitech c310 has a focal width of 2mm
     * @param f focal width
     * @param w width of object (IN INCHES)!
     * @param p width in pixels
     * @return the distance
     * @author aryansinha
     */
    private double formulateDistance(double f, double w, double p) {
        distance = ((f*w)/p);
        return distance;
    }

    private synchronized void setDirection(double leftValue, double middleValue, double rightValue) {
        boolean leftBool = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean middleBool = middleValue > PERCENT_COLOR_THRESHOLD;
        boolean rightBool = rightValue > PERCENT_COLOR_THRESHOLD;

        if( rightBool ) {
            direction = Direction.RIGHT;
            telemetry.addData( "Location",  "right" );
        } else if( leftBool ) {
            direction = Direction.LEFT;
            telemetry.addData( "Location", "left" );
        } else if( middleBool ) {
            direction = Direction.IN_FRONT;
            telemetry.addData( "Location", "middle" );
        } else {
            direction = Direction.NOT_FOUND;
            telemetry.addData( "Location", "not found" );
        }
    }

    private synchronized void setDirectionAndDistance(double leftValue, double middleValue, double rightValue, Mat mat) {
        Mat edges = new Mat();
        Mat hierarchy = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[0];
        try {
            // Copied threshold values from an example.
            Imgproc.Canny(mat, edges, 100, 300);
            Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            contoursPoly  = new MatOfPoint2f[contours.size()];
            Rect[] boundRect = new Rect[contours.size()];
            if (contours.size() > 0) {
              contoursPoly = new MatOfPoint2f[contours.size()];
              boundRect = new Rect[contours.size()];
              for (int i = 0; i < contours.size(); i++) {
                  contoursPoly[i] = new MatOfPoint2f();
                  Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                  boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
              }
            }
            double left_x = 0.25 * FRAME_WIDTH;
            double right_x = 0.75 * FRAME_WIDTH;

            for (int i = 0; i != boundRect.length; i++) {
              if (boundRect[i].x < left_x)
                  left = true;
              if (boundRect[i].x + boundRect[i].width > right_x)
                  right = true;

              // draw red bounding rectangles on mat
              // the mat has been converted to HSV so we need to use HSV as well
              Imgproc.rectangle(mat, boundRect[i], new Scalar(255, 0, 0));
           }
        } finally {
          edges.release();
          hierarchy.release();
        }

        //is it left or is it right 🤔
        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneFront = middleValue > PERCENT_COLOR_THRESHOLD;

        if (contours.size() == 0) {
            direction = NOT_FOUND;
            telemetry.addData("Skystone Location", "not found");
        } else if (stoneLeft || left) {
            direction = LEFT;
            telemetry.addData("Skystone Location", "right");
            telemetry.addData("Distance", formulateDistance(2, 2, getWidth(contoursPoly[0])));
        } else if (stoneRight || right){
            direction = RIGHT;
            distance = formulateDistance(2, 2, getWidth(contoursPoly[0]));
            telemetry.addData("Skystone Location", "left");
            telemetry.addData("Distance", distance);
        } else if (stoneFront) {
            direction = IN_FRONT;
            distance = formulateDistance(2, 2, getWidth(contoursPoly[0]));
            telemetry.addData("Skystone Location", "in front");
            telemetry.addData("Distance", distance);
        }

        telemetry.addData("number of Contours", contoursPoly.length);

        for (int i = 0; i< contoursPoly.length; i ++) {
            distance = formulateDistance(2, 2, getWidth(contoursPoly[i]));
            telemetry.addData("distance from contour", "for element " + (i + 1) + "of size " + contoursPoly[0].size() + "=" + distance);
        }
        telemetry.update();
    }

    private double getWidth(MatOfPoint2f currentContour) {
        Rect rectangle = Imgproc.boundingRect(currentContour);
        double width = rectangle.width;
        return (width);
    }

    private void logDebugData(double leftValue, double middleValue, double rightValue, Mat left, Mat right, Mat middle) {
        //Stats for nerds
        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(middleValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
        telemetry.addData("Distance", distance);
        telemetry.update();
    }
}