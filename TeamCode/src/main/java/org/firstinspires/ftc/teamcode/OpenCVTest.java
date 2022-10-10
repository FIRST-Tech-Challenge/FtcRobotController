package org.firstinspires.ftc.teamcode;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.sql.Array;
import java.util.ArrayList;
import java.util.Arrays;

public class OpenCVTest extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        RIGHT,
        MIDDLE,
        NOT_FOUND
    }
    private Location location;

    // replace with real values later
    static final Rect TARGET_LOCATION = new Rect(
            new Point(60, 35),
            new Point(120, 75));


    public OpenCVTest(Telemetry t) {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        telemetry = t;
    }


    @Override
    public Mat processFrame(Mat input) {
//        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
//        Scalar lowHSV = new Scalar(23, 50, 70);
//        Scalar highHSV = new Scalar(32, 255, 255);

//        Core.inRange(mat, lowHSV, highHSV, mat);

        //
//
//        Mat m = new Mat(5, 10, CvType.CV_8UC1, new Scalar(251));
//        m.row(1).setTo(new Scalar(252));
//        m.row(2).setTo(new Scalar(253));
//        m.row(3).setTo(new Scalar(254));
//        m.row(4).setTo(new Scalar(255));
//
//        System.out.println( m.dump() );
//        ArrayList<Double> test = new ArrayList<Double>();
//        for ( int i=0; i<mat.rows(); ++i )
//        {
//            for ( int j=0; j<mat.cols(); ++j )
//            {
//                double[] d = mat.get(i, j);
//
//                test.add(d[0]);
//                arr[j][i] = tmp[0];
//            }
//        }

        Mat target = mat.submat(TARGET_LOCATION);

        // turn t arget into three channels
        ArrayList<Mat> a = new ArrayList<Mat>();
        Core.split(target, a);

        Mat blue = a.get(0);
        Mat green = a.get(1);
        Mat red = a.get(2);

        double blueVal = Core.sumElems(blue).val[0] / TARGET_LOCATION.area() / 255; // get average
        double greenVal = Core.sumElems(green).val[0] / TARGET_LOCATION.area() / 255; // get average
        double redVal = Core.sumElems(red).val[0] / TARGET_LOCATION.area() / 255; // get average

        blue.release();
        green.release();
        red.release();

        telemetry.addData("Target blue value", (int) Core.sumElems(blue).val[0]);
        telemetry.addData("Target green value", (int) Core.sumElems(green).val[0]);
        telemetry.addData("Target red value", (int) Core.sumElems(red).val[0]);

        telemetry.addData("Target blue percentage", Math.round(blueVal * 100) + "%");
        telemetry.addData("Target green percentage", Math.round(greenVal * 100) + "%");
        telemetry.addData("Target red percentage", Math.round(redVal * 100) + "%");

        boolean left = redVal > 0.75; // purple ish
        boolean middle = blueVal > 0.85; // testing - need to tune - blue ish
        boolean right = redVal > 0.96; // orange ish - higher red value

        if (left) {
            location = Location.LEFT;
            telemetry.addData("Cone Location", "LEFT");
        }
        else if (right) {
            location = Location.RIGHT;
            telemetry.addData("Cone Location", "RIGHT");
        } else if (middle) {
            location = Location.MIDDLE;
            telemetry.addData("Cone Location", "MIDDLE");
        } else {
            location = Location.NOT_FOUND;
            telemetry.addData("Cone Location", "NOT FOUND");
        }

        telemetry.update();

//        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        // modify colors
        Scalar colorLeft = new Scalar(255, 0, 0);
        Scalar colorRight = new Scalar(0, 255, 0);
        Scalar colorMiddle = new Scalar(0, 0, 255);
        Scalar colorNone = new Scalar(255, 255, 255);

        if (right) {
            Imgproc.rectangle(mat, TARGET_LOCATION, colorRight);
        } else if (left) {
            Imgproc.rectangle(mat, TARGET_LOCATION, colorLeft);
        } else if (middle) {
            Imgproc.rectangle(mat, TARGET_LOCATION, colorMiddle);
        } else {
            Imgproc.rectangle(mat, TARGET_LOCATION, colorNone);
        }

        return mat;
    }

    public Location getLocation() {
        return location;
    }

}


