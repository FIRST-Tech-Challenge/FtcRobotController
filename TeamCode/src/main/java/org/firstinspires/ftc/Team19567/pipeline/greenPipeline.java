package org.firstinspires.ftc.Team19567.pipeline;

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
 * The {@link #THRESHOLD} value for detecting green is situated in {@link org.firstinspires.ftc.Team19567.util.Utility_Constants Utility_Constants.java}
 *<br><br>
 * TODO: Improve consistency in the future
 */

public class greenPipeline extends OpenCvPipeline {
    private Mat output = new Mat();
    private static final double width = 544;
    private static final double height = 288;
    private static final double margin = 10;
    private static final double one_square = (width-4*margin)/3;
    private static final Rect LEFT_SQUARE = new Rect(
            new Point(margin,margin), new Point(margin+one_square,height-margin)
    );

    private static final Rect RIGHT_SQUARE = new Rect(
            new Point(2*margin+one_square,margin), new Point(2*(margin+one_square),height-margin)
    );

    private static final Rect RIGHTEST_SQUARE = new Rect(
            new Point(3*margin+2*one_square,margin), new Point( 3*(margin+one_square),height-margin)
    );

    private static double THRESHOLD = 0.025;

    Telemetry telemetry;

    /**
     *
     * @param t Telemetry used to broadcast values to the driver station
     */
    public greenPipeline(Telemetry t) {
        telemetry = t;
    }

    private Scalar lowHSV = new Scalar(36, 50, 70);
    private Scalar highHSV = new Scalar(89, 255, 255);
    private double firstConf = 0.0;
    private double secondConf = 0.0;
    private double thirdConf = 0.0;
    private boolean tseFirst = false;
    private boolean tseSecond = false;
    private boolean tseThird = false;
    Scalar detectedColor = new Scalar(0,255,0);
    Scalar none = new Scalar(255,0,0);

    private LOCATION location = LOCATION.ALLIANCE_THIRD;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input,output,Imgproc.COLOR_RGB2HSV); //Converts input from RGB to HSV (which is better for thresholding)
        //telemetry.addData("Pipeline Status","Setup Complete");

        Core.inRange(output,lowHSV,highHSV,output); //Binarizes the image, with all HSV values between the low and high
        //telemetry.addData("Pipeline Status","InRange Conv. Completed");

        Mat first = output.submat(LEFT_SQUARE);
        Mat second = output.submat(RIGHT_SQUARE);
        Mat third = output.submat(RIGHTEST_SQUARE);
        //telemetry.addData("Pipeline Status","Submats Computed");

        firstConf = Core.sumElems(first).val[0] / LEFT_SQUARE.area()/255;
        secondConf = Core.sumElems(second).val[0] / RIGHT_SQUARE.area()/255;
        thirdConf = Core.sumElems(third).val[0] / RIGHTEST_SQUARE.area()/255;

        //telemetry.addData("Pipeline Status","Confidences Ascertained");

        first.release();
        second.release();

        /* telemetry.addData("Left",(int) Core.sumElems(first).val[0]);
        telemetry.addData("Right",(int) Core.sumElems(second).val[0]);
        telemetry.addData("LeftP(%.2f)",firstConf*100);
        telemetry.addData("RightP(%.2f)",secondConf*100);
        telemetry.addData("Telemetry Status","Values Broadcasted via Telemetry"); */

        tseFirst = firstConf > THRESHOLD;
        tseSecond = secondConf > THRESHOLD;
        tseThird = thirdConf > THRESHOLD;

        /* telemetry.addData("Yop?",tseFirst);
        telemetry.addData("Yop2?",tseSecond);
        telemetry.addData("Yop3?",tseThird); */

        if(tseFirst) {
            location = LOCATION.ALLIANCE_FIRST;
            /* telemetry.addData("Level","First");
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
        //telemetry.addData("OpenCV Status","Location Decided");

        Imgproc.cvtColor(output, output, Imgproc.COLOR_GRAY2RGB);

        /* telemetry.addData("OpenCV Status","Final Conversion + Color Complete");
        telemetry.update(); */

        Imgproc.rectangle(output,LEFT_SQUARE,location==LOCATION.ALLIANCE_FIRST? detectedColor:none);
        Imgproc.rectangle(output,RIGHT_SQUARE,location==LOCATION.ALLIANCE_SECOND? detectedColor:none);
        Imgproc.rectangle(output,RIGHTEST_SQUARE,(location==LOCATION.ALLIANCE_THIRD || location==LOCATION.NO_ALLIANCE)? detectedColor:none);
        System.gc();
        /* telemetry.addData("OpenCV Status","Rectangles Drawn");
        telemetry.addData("Input Frame Size",input.rows()+" x "+input.cols());
        telemetry.addData("Output Frame Size", output.rows()+" x "+output.cols());
        telemetry.update(); */

        return output;
    }

    /**
     * @return The {@link #location LOCATION} that the pipeline has currently <br>
     * detected the TSE to be in
     */
    public LOCATION getLocation() {
        return location;
    }
}