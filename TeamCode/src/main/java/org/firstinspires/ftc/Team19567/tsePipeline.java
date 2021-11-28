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
        new Point(50,100), new Point(350,250)
    );

    private static final Rect RIGHT_SQUARE = new Rect( //Same here
            new Point(400,100), new Point(700,250)
    );

    private static final Rect FAIYOI_SQUARE = new Rect(
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
        if(input.empty()) {
            return input;
        }
        Imgproc.cvtColor(input,output,Imgproc.COLOR_RGB2HSV);
        input.release();

        Scalar lowHSV = new Scalar(120, 25, 62);
        Scalar highHSV = new Scalar(120,100,54);

        Core.inRange(output,lowHSV,highHSV,output);

        Mat left = output.submat(LEFT_SQUARE);
        Mat right = output.submat(RIGHT_SQUARE);

        double leftConf = Core.sumElems(left).val[0] / LEFT_SQUARE.area()/255;
        double rightConf = Core.sumElems(right).val[0] / RIGHT_SQUARE.area()/255;

        left.release();
        right.release();

        telemetry.addData("Left",(int) Core.sumElems(left).val[0]);
        telemetry.addData("Right",(int) Core.sumElems(right).val[0]);
        telemetry.addData("LeftP(%.2f)",leftConf*100);
        telemetry.addData("RightP(%.2f)",rightConf*100);

        boolean tseLeft = leftConf > THRESHOLD;
        boolean tseRight = rightConf > THRESHOLD;

        if(tseLeft) {
            location = LOCATION.ALLIANCE_FIRST;
            telemetry.addData("Level","First");
        }
        else if(tseRight) {
            location = LOCATION.ALLIANCE_SECOND;
            telemetry.addData("Level","Second");
        }
        else {
            location = LOCATION.ALLIANCE_THIRD;
            telemetry.addData("Level","Third");
        }

        telemetry.update();

        Imgproc.cvtColor(output, output, Imgproc.COLOR_GRAY2RGB);

        Scalar thirdLevel = new Scalar(255,0,0);
        Scalar secondLevel = new Scalar(0,255,0);
        Scalar firstLevel = new Scalar(0,0,255);
        Scalar none = new Scalar(0,0,0);

        Imgproc.rectangle(output,LEFT_SQUARE,tseLeft? firstLevel:none);
        Imgproc.rectangle(output,RIGHT_SQUARE,tseRight? secondLevel:none);
        Imgproc.rectangle(output,FAIYOI_SQUARE,(!tseLeft && !tseRight)? thirdLevel:none);

        return output;
    }

    public LOCATION getLocation() {
        return location;
    }
}