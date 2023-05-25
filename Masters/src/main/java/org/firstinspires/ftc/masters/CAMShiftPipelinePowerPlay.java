package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.Video;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Collections;

public class CAMShiftPipelinePowerPlay extends OpenCvPipeline {

    public String data;


    public enum DetectedObject {
        BLUE_CONE,
        RED_CONE,
        POLE,
        THE_MADNESS_OF_CTHULHU
    }

    public double heightWidthRatio;
    public int numCones;

    public enum ConeOrientation {
        UPRIGHT,
        TIPPED
    }

    ConeOrientation orientation;

    Mat hsv_roi = new Mat();
    Mat mask = new Mat();

    // hardcode the initial location of window
    private final Rect trackWindow = new Rect(150, 60, 63, 125);
    Telemetry telemetry;
    private final TelemetryPacket packet;

    public CAMShiftPipelinePowerPlay(Telemetry telemetry, TelemetryPacket packet) {
        this.telemetry = telemetry;
        this.packet = packet;
    }

    Mat region_a;
    Mat region_b;

    Mat LAB = new Mat();
    Mat A = new Mat();
    Mat B = new Mat();

    public volatile DetectedObject detectedObject = DetectedObject.THE_MADNESS_OF_CTHULHU;

    Point topLeftPoint;
    Point bottomRightPoint;

    int avg_a = 0;
    int avg_b = 0;

    int difFromPoleThresh = 190; //b
    int difFromRedConeThresh = 180; //a
    int difFromBlueConeThresh = 85; //b

    int difFromPole; //b
    int difFromRedCone; //a
    int difFromBlueCone; //b

    public Point center;
    public Size size;

    TermCriteria term_crit;

    RotatedRect rot_rect;

    Mat roi, roi_hist, hsv, dst;
    MatOfFloat range = new MatOfFloat(0, 256);
    MatOfInt histSize = new MatOfInt(180);
    MatOfInt channels = new MatOfInt(0);

    Point[] points;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    void inputToLAB(Mat input) {

        Imgproc.cvtColor(input, LAB, Imgproc.COLOR_RGB2Lab);
        Core.extractChannel(LAB, A, 1);
        Core.extractChannel(LAB, B, 2);
    }

    @Override
    public Mat processFrame(Mat input) {

        roi = input.submat(trackWindow);
        Imgproc.cvtColor(roi, hsv_roi, Imgproc.COLOR_BGR2HSV);
        Core.inRange(hsv_roi, new Scalar(0, 60, 32), new Scalar(180, 255, 255), mask);

        roi_hist = new Mat();
        Imgproc.calcHist(Collections.singletonList(hsv_roi), channels, mask, roi_hist, histSize, range);
        Core.normalize(roi_hist, roi_hist, 0, 255, Core.NORM_MINMAX);

        term_crit = new TermCriteria(TermCriteria.EPS | TermCriteria.COUNT, 100, .1);


        hsv = new Mat();
        dst = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
        Imgproc.calcBackProject(Collections.singletonList(hsv), channels, roi_hist, dst, range, 1);

        rot_rect = Video.CamShift(dst, trackWindow, term_crit);

        points = new Point[4];
        rot_rect.points(points);
        for (int i = 0; i < 4 ;i++) {
            Imgproc.line(input, points[i], points[(i+1)%4], new Scalar(255, 0, 0),2);
        }

        data = rot_rect.toString();
        telemetry.addData("rot_rect: ", rot_rect.toString());
        packet.put("rot_rect: ", rot_rect.toString());

        center = rot_rect.center;
        size = rot_rect.size;
        double angle = rot_rect.angle;

        inputToLAB(input);

        if (angle > 165 && angle < 195) {
            orientation = ConeOrientation.UPRIGHT;
        } else if (angle > 75 && angle < 105) {
            orientation = ConeOrientation.TIPPED;
        }

        telemetry.addData("Orientation: ", orientation);
        packet.put("Orientation: ", orientation);

        if (orientation == ConeOrientation.UPRIGHT) {
            topLeftPoint = new Point(center.x-(size.width/4),center.y-(size.height/4));
            bottomRightPoint = new Point(center.x+(size.width/4),center.y+(size.height/4));
        } else if (orientation == ConeOrientation.TIPPED) {
            topLeftPoint = new Point(center.x-(size.height/4),center.y-(size.width/4));
            bottomRightPoint = new Point(center.x+(size.height/4),center.y+(size.width/4));
        }



        try {
            region_a = A.submat(new Rect(topLeftPoint, bottomRightPoint));
            region_b = B.submat(new Rect(topLeftPoint, bottomRightPoint));

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    topLeftPoint, // First point which defines the rectangle
                    bottomRightPoint, // Second point which defines the rectangle
                    new Scalar(0,0,255), // The color the rectangle is drawn in (blue)
                    2); // Thickness of the rectangle lines//  Block of code to try

            avg_a = (int) Core.mean(region_a).val[0];
            avg_b = (int) Core.mean(region_b).val[0];

            telemetry.addData("avg_a: ", avg_a);
            telemetry.addData("avg_b: ", avg_b);
            packet.put("avg_a: ", avg_a);
            packet.put("avg_b: ", avg_b);



            difFromPole = Math.abs(difFromPoleThresh - avg_b);
            difFromRedCone = Math.abs(difFromRedConeThresh - avg_a);
            difFromBlueCone = Math.abs(difFromBlueConeThresh - avg_b);

            if (difFromPole < difFromRedCone && difFromPole < difFromBlueCone) {
                detectedObject = DetectedObject.POLE;
            } else if (difFromBlueCone < difFromPole && difFromBlueCone < difFromRedCone) {
                detectedObject = DetectedObject.BLUE_CONE;
            } else if (difFromRedCone < difFromBlueCone) {
                detectedObject = DetectedObject.RED_CONE;
            }

            telemetry.addData("Detected Object: ", detectedObject);
            packet.put("Detected Object: ", detectedObject);


        }
        catch(Exception e) {

            telemetry.addData("Region out of bounds","");//  Block of code to handle errors
            detectedObject = DetectedObject.THE_MADNESS_OF_CTHULHU;
        }

        heightWidthRatio = size.height/size.width;
        switch (detectedObject) {
            case POLE:
                telemetry.addData("Center X",center.x);
                telemetry.addData("Pole H->W Ratio", heightWidthRatio);
                packet.put("Center X",center.x);
                packet.put("Pole H->W Ratio", heightWidthRatio);

                break;
            case RED_CONE:
            case BLUE_CONE:
                telemetry.addData("Cone H->W Ratio", heightWidthRatio);
                packet.put("Cone H->W Ratio", heightWidthRatio);
                if (heightWidthRatio >= 1.70 && heightWidthRatio <= 1.90) {
                    numCones = 1;
                } else if (heightWidthRatio >= 1.90 && heightWidthRatio <= 2.20) {
                    numCones = 2;
                } else if (heightWidthRatio >= 2.20 && heightWidthRatio <= 2.45) {
                    numCones = 3;
                } else if (heightWidthRatio >= 2.45 && heightWidthRatio <= 2.75) {
                    numCones = 4;
                } else if (heightWidthRatio >= 2.75 && heightWidthRatio <= 3.00) {
                    numCones = 5;
                }
                telemetry.addData("Num Cones: ",numCones);
                packet.put("Num Cones: ",numCones);
                break;

            case THE_MADNESS_OF_CTHULHU:
                break;
        }


        dashboard.sendTelemetryPacket(packet);


        telemetry.update();



        return input;
    }
}