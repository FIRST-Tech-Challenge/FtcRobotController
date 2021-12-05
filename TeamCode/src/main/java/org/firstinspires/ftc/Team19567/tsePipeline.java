package org.firstinspires.ftc.Team19567;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class tsePipeline extends OpenCvPipeline {
    private Mat output = new Mat();
    private static final Rect LEFT_SQUARE = new Rect(
        new Point(60,60), new Point(180,420)
    );

    private static final Rect RIGHT_SQUARE = new Rect(
            new Point(240,60), new Point(360,420)
    );

    private static final Rect RIGHTEST_SQUARE = new Rect(
      new Point(420,60), new Point( 540,420)
    );

    private static double THRESHOLD = 0.4;

    Telemetry telemetry;

    public tsePipeline(Telemetry t) {
        telemetry = t;
    }

    public enum LOCATION {
        ALLIANCE_FIRST,
        ALLIANCE_SECOND,
        ALLIANCE_THIRD
    }

    private Scalar lowHSV = new Scalar(120, 25, 62);
    private Scalar highHSV = new Scalar(120,100,54);
    private double firstConf = 0.0;
    private double secondConf = 0.0;
    private boolean tseFirst = false;
    private boolean tseSecond = false;
    Scalar detectedColor = new Scalar(0,255,0);
    Scalar none = new Scalar(255,0,0);

    private LOCATION location;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input,output,Imgproc.COLOR_RGB2HSV);
        telemetry.addData("Pipeline Status","Setup Complete");
        telemetry.update();



        Core.inRange(output,lowHSV,highHSV,output);
        telemetry.addData("Pipeline Status","InRange Conv. Completed");
        telemetry.update();

        Mat first = output.submat(LEFT_SQUARE);
        Mat second = output.submat(RIGHT_SQUARE);
        telemetry.addData("Pipeline Status","Submats Computed");
        telemetry.update();

        firstConf = Core.sumElems(first).val[0] / LEFT_SQUARE.area()/255;
        secondConf = Core.sumElems(second).val[0] / RIGHT_SQUARE.area()/255;

        telemetry.addData("Pipeline Status","Confidences Ascertained");
        telemetry.update();

        first.release();
        second.release();

        telemetry.addData("Left",(int) Core.sumElems(first).val[0]);
        telemetry.addData("Right",(int) Core.sumElems(second).val[0]);
        telemetry.addData("LeftP(%.2f)",firstConf*100);
        telemetry.addData("RightP(%.2f)",secondConf*100);
        telemetry.addData("Telemetry Status","Values Broadcasted via Telemetry");
        telemetry.update();

        tseFirst = firstConf > THRESHOLD;
        tseSecond = secondConf > THRESHOLD;

        telemetry.addData("Yop?",tseFirst);
        telemetry.addData("Yop2?",tseSecond);
        telemetry.update();

        if(tseFirst) {
            location = LOCATION.ALLIANCE_FIRST;
            telemetry.addData("Level","First");
        }
        else if(tseSecond) {
            location = LOCATION.ALLIANCE_SECOND;
            telemetry.addData("Level","Second");
        }
        else {
            location = LOCATION.ALLIANCE_THIRD;
            telemetry.addData("Level","Third");
        }
        telemetry.addData("OpenCV Status","Location Decided");
        telemetry.update();

        Imgproc.cvtColor(output, output, Imgproc.COLOR_GRAY2RGB);


        telemetry.addData("OpenCV Status","Final Conversion + Color Complete");
        telemetry.update();

        Imgproc.rectangle(output,LEFT_SQUARE,tseFirst? detectedColor:none);
        Imgproc.rectangle(output,RIGHT_SQUARE,tseSecond? detectedColor:none);
        Imgproc.rectangle(output,RIGHTEST_SQUARE,(!tseFirst && !tseSecond)? detectedColor:none);
        telemetry.addData("OpenCV Status","Rectangles Drawn");
        telemetry.update();
        System.gc();
        telemetry.addData("Input Frame Size",input.rows()+" x "+input.cols());
        telemetry.addData("Output Frame Size", output.rows()+" x "+output.cols());
        telemetry.update();

        return output;
    }

    public LOCATION getLocation() {
        return location;
    }
}