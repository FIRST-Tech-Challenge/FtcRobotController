package org.firstinspires.ftc.Team19567.pipeline;

import static org.firstinspires.ftc.Team19567.util.Utility_Constants.OPENCV_THRESHOLD;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * A pipeline to detect green (aka the Team Shipping Element). <br>
 * Minor consistency issues when the TSE is leaning out of frame <br><br>
 * This pipeline works via thresholding with HSV values<br>
 * The {@link org.firstinspires.ftc.Team19567.util.Utility_Constants#OPENCV_THRESHOLD THRESHOLD} value for detecting green is situated in {@link org.firstinspires.ftc.Team19567.util.Utility_Constants Utility_Constants.java}
 *<br><br>
 * TODO: Improve consistency in the future
 */

public class greenPipeline extends OpenCvPipeline {
    /** Output Mat (frame) */
    private Mat output = new Mat();
    /** Width of the camera's streamed viewport */
    private static final double width = 544;
    /** Height of the camera's streamed viewport */
    private static final double height = 288;
    /** Margin, in px, between the ROI (regions of interest) and the viewport's edges */
    private static final double margin = 10;
    /** Width of one ROI */
    private static final double one_square = (width-4*margin)/3;
    /** Leftmost ROI */
    private static final Rect LEFT_SQUARE = new Rect( //The ROIs are specified as Rects, or rectangles, which can be constructed from two points
            new Point(margin,margin), new Point(margin+one_square,height-margin) //The two points represent the upper-left and bottom-right points, respectively
    ); //You can try to figure out the math here yourself
    /** Middle ROI */
    private static final Rect RIGHT_SQUARE = new Rect(
            new Point(2*margin+one_square,margin), new Point(2*(margin+one_square),height-margin)
    );
    /** Rightmost ROI */
    private static final Rect RIGHTEST_SQUARE = new Rect(
            new Point(3*margin+2*one_square,margin), new Point( 3*(margin+one_square),height-margin)
    );
    /** Telemetry that will be used to broadcast values to the driver station */
    Telemetry telemetry; //The telemetry won't been assigned any value until the constructor

    /** @param telemetry Telemetry used to broadcast values to the driver station */
    public greenPipeline(Telemetry telemetry) {
        this.telemetry = telemetry; //this.telemetry => this object's telemetry; differentiates it from the telemetry passed as a constructor parameter
    }

    /** "Lower bound" for HSV value for green */
    private final Scalar lowHSV = new Scalar(36, 50, 70);
    /* You can obtain these values by searching them up,
    or by experimenting with FTC Dashboard (configuring various HSV bounds) */
	/** "Upper bound" for HSV value for green */
    private final Scalar highHSV = new Scalar(89, 255, 255);
    /** "Perfect green" */
    Scalar detectedColor = new Scalar(0,255,0);
    /** "Perfect red" */
    Scalar none = new Scalar(255,0,0);

    /** Detected location of the TSE */
    private LOCATION location = LOCATION.ALLIANCE_THIRD;
    /**
     * Function to process each frame the pipeline is active.
     * @param input The input frame
     * @return The output frame
     */
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input,output,Imgproc.COLOR_RGB2HSV); //Converts input from RGB to HSV (which is better for thresholding)
        /* telemetry.addData("Pipeline Status","Setup Complete");
        telemetry.update(); //Debug telemetry */

        Core.inRange(output,lowHSV,highHSV,output); //Binarizes the image, with all HSV values between the low and high as white and everything else as black
        /* telemetry.addData("Pipeline Status","InRange Conv. Completed");
        telemetry.update() //Debug telemetry */

        Mat first = output.submat(LEFT_SQUARE);
        Mat second = output.submat(RIGHT_SQUARE);
        Mat third = output.submat(RIGHTEST_SQUARE);
        /* telemetry.addData("Pipeline Status","Submats Computed");
        /telemetry.update(); //Debug telemetry */

        /*
        Variable representing the amount of "green" detected in the first ROI;
        The variable is obtained by adding up all of the pixels in the first ROI. All black pixels have "value zero" and do not contribute to the sum.
        Thus, the sum is just all of the white pixels, or detected "green," in the first ROI. the Core.sumElems() function returns a scalar containing
        a few values. However, only the first one is what we want (the sum), so we get its corresponding array, .val, and access the value
        at its first index, [0]. Then, we divide this by the amount of pixels in the first ROI to get the percentage that is green, and again by 255,
        because white pixels in OpenCV have sum 255, so in reality the amount of green pixels is sumElems(first).val[0]/255.
        All of these operations give us the percentage of the ROI that is green.
        Technically, we could just use the raw sum, but this method is more intuitive, as a raw sum conveys no meaning.
         */
        double firstConf = Core.sumElems(first).val[0] / LEFT_SQUARE.area() / 255;
        double secondConf = Core.sumElems(second).val[0] / RIGHT_SQUARE.area() / 255; //Self explanatory
        double thirdConf = Core.sumElems(third).val[0] / RIGHTEST_SQUARE.area() / 255; //Self explanatory

        /* telemetry.addData("Pipeline Status","Confidences Ascertained");
        telemetry.update(); //Debug telemetry */

        first.release(); //Deallocates the memory of the "first" variable. This is *NECESSARY* to prevent a memory leak
        //where OpenCV tries to allocate too much memory. See https://www.chiefdelphi.com/t/a-warning-opencv-memory-leaks/158458 for more info.
        second.release();
        third.release();

        /*
        Debug telemetry

        telemetry.addData("Left",(int) Core.sumElems(first).val[0]);
        telemetry.addData("Right",(int) Core.sumElems(second).val[0]);
        telemetry.addData("LeftP(%.2f)",firstConf*100);
        telemetry.addData("RightP(%.2f)",secondConf*100);
        telemetry.addData("Telemetry Status","Values Broadcasted via Telemetry");
        */

        //Variable representing whether or not a TSE has been detected in the first ROI
        //The comparison symbol returns a boolean, just like in a conditional statement, which is why this syntax works (should be pretty self explanatory)
        boolean tseFirst = firstConf > OPENCV_THRESHOLD;
        boolean tseSecond = secondConf > OPENCV_THRESHOLD;
        boolean tseThird = thirdConf > OPENCV_THRESHOLD;

        //If statements for operations depending on whether or not a TSE has been detected in each ROI
        //Note that the else statement will run if multiple TSEs have been detected OR if none have been detected
        if(tseFirst) {
            location = LOCATION.ALLIANCE_FIRST; //Sets the location to LOCATION.ALLIANCE_FIRST
            /* telemetry.addData("Level","First"); //Debug telemetry
            telemetry.update(); */
        }
        else if(tseSecond) {
            location = LOCATION.ALLIANCE_SECOND;
            /* telemetry.addData("Level","Second");
            telemetry.update(); */
        }
        else if(tseThird) {
            location = LOCATION.ALLIANCE_THIRD;
        }
        else {
            location = LOCATION.NO_ALLIANCE;
            /* telemetry.addData("Level","Third");
            telemetry.update(); */
        }
        /* telemetry.addData("OpenCV Status","Location Decided"); //Debug telemetry
        telemetry.update() */

        Imgproc.cvtColor(output, output, Imgproc.COLOR_GRAY2RGB); //Converts the image back to RGB

        /* telemetry.addData("OpenCV Status","Final Conversion + Color Complete"); //Debug Telemetry
        telemetry.update(); */

        /*
        Draws a rectangle around each ROI
        Note the use of the ternary statement that determines whether to draw with the "detectedColor" (green)
        or "none" (red). It will execute the first statement before the colon if the statement before the question mark is true, and vice versa.
         */
        Imgproc.rectangle(output,LEFT_SQUARE,location==LOCATION.ALLIANCE_FIRST? detectedColor:none);
        Imgproc.rectangle(output,RIGHT_SQUARE,location==LOCATION.ALLIANCE_SECOND? detectedColor:none);
        Imgproc.rectangle(output,RIGHTEST_SQUARE,(location==LOCATION.ALLIANCE_THIRD || location==LOCATION.NO_ALLIANCE)? detectedColor:none);

        /*
        Important debug telemetry:

        telemetry.addData("OpenCV Status","Rectangles Drawn");
        telemetry.addData("Input Frame Size",input.rows()+" x "+input.cols());
        telemetry.addData("Output Frame Size", output.rows()+" x "+output.cols());
        telemetry.update();
        */

        return output; //Returns the output to be drawn on the DS viewport
    }

    /**
     * @return The {@link #location LOCATION} that the pipeline has currently <br>
     * detected the TSE to be in
     */
    public LOCATION getLocation() {
        return location;
    }
}