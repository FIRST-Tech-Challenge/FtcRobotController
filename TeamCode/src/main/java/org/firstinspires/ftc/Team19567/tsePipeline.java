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

    private static final Rect LEFT_SQUARE = new Rect( //Placeholder, TBD
        new Point(50,100), new Point(250,250)
    );

    private static final Rect RIGHT_SQUARE = new Rect( //Same here
            new Point(400,100), new Point(200,250)
    );

    private static final Rect RIGHTEST_SQUARE = new Rect(
      new Point(25,25), new Point(125,50)
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

    private LOCATION location;

    @Override
    public Mat processFrame(Mat input) {
        output.release();
        Imgproc.cvtColor(input,output,Imgproc.COLOR_RGB2HSV);
        input.release();
        telemetry.addData("Pipeline Status","Setup Complete");
        telemetry.update();

        Scalar lowHSV = new Scalar(120, 25, 62);
        Scalar highHSV = new Scalar(120,100,54);

        Core.inRange(output,lowHSV,highHSV,output);
        telemetry.addData("Pipeline Status","InRange Conv. Completed");
        telemetry.update();

        Mat first = output.submat(LEFT_SQUARE);
        Mat second = output.submat(RIGHT_SQUARE);
        telemetry.addData("Pipeline Status","Submats Computed");
        telemetry.update();

        double firstConf = Core.sumElems(first).val[0] / LEFT_SQUARE.area()/255;
        double secondConf = Core.sumElems(second).val[0] / RIGHT_SQUARE.area()/255;

        telemetry.addData("Pipeline Status","Confidences Ascertained");
        telemetry.update();

        first.release();
        second.release();

        telemetry.addData("Left",(int) Core.sumElems(first).val[0]);
        telemetry.addData("Right",(int) Core.sumElems(second).val[0]);
        telemetry.addData("LeftP(%.2f)",firstConf*100);
        telemetry.addData("RightP(%.2f)",secondConf*100);
        telemetry.addData("OpenCV Status","Values Broadcasted via Telemetry");
        telemetry.update();

        boolean tseFirst = firstConf > THRESHOLD;
        boolean tseSecond = secondConf > THRESHOLD;

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

        Scalar detectedColor = new Scalar(0,255,0);
        Scalar none = new Scalar(255,0,0);
        telemetry.addData("OpenCV Status","Final Conversion + Color Complete");
        telemetry.update();

        Imgproc.rectangle(output,LEFT_SQUARE,tseFirst? detectedColor:none);
        Imgproc.rectangle(output,RIGHT_SQUARE,tseSecond? detectedColor:none);
        Imgproc.rectangle(output,RIGHTEST_SQUARE,(!tseFirst && !tseSecond)? detectedColor:none);
        telemetry.addData("OpenCV Status","Rectangles Drawn");
        telemetry.update();
        System.gc();

        return output;
    }

    public LOCATION getLocation() {
        return location;
    }
}