package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.teaminfo.TeamColor;
import org.firstinspires.ftc.teamcode.components.teaminfo.TeamInfo;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class AutonDetector extends OpenCvPipeline {
    Telemetry telemetry;
    public AutonDetector(Telemetry t){ this.telemetry = t; }

    public enum DetectMode {CONE, POLE, NONE}

    public DetectMode detectMode = DetectMode.NONE;
    private Mat mat = new Mat();
    private int screenWidth;

    public Rect target = new Rect();

    @Override
    public Mat processFrame(Mat input) {
        screenWidth = input.width();
        return autonDetection(input);
    }

    private Mat autonDetection(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Mat thresh = new Mat();
        if(detectMode == DetectMode.POLE) thresh = yellowThresh();
        else if (detectMode == DetectMode.CONE){
            if(TeamInfo.teamColor == TeamColor.RED) thresh = redThresh();
            else if(TeamInfo.teamColor == TeamColor.BLUE) thresh = blueThresh();
        }

        Imgproc.erode(thresh, thresh, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

        List<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(thresh, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        contours.removeIf(c -> Imgproc.boundingRect(c).height < 20 || Imgproc.boundingRect(c).area() < 50);
        Imgproc.drawContours(input, contours, -1, new Scalar(255, 255, 255));

        if(!contours.isEmpty()) {
            MatOfPoint biggestWidth = Collections.max(contours, Comparator.comparingDouble(t0 -> Imgproc.boundingRect(t0).width));
            target = Imgproc.boundingRect(biggestWidth);

            Imgproc.rectangle(input, target, new Scalar(255, 0, 0), 2);
            Imgproc.circle(input, new Point(target.x + (target.width/2.0), target.y + (target.height/2.0)), 1, new Scalar(255, 0, 255), 3);
        }

        contours.clear();
        mat.release();
        thresh.release();

        return input;
    }

    private Mat redThresh() {
        Scalar red1_lowHSV = new Scalar (0, 64, 20);
        Scalar red1_highHSV = new Scalar (10, 255, 255);
        Mat thresh1 = new Mat();

        Scalar red2_lowHSV = new Scalar(160, 64, 20);
        Scalar red2_highHSV = new Scalar (180, 255, 255);
        Mat thresh2 = new Mat();

        Core.inRange(mat, red1_lowHSV, red1_highHSV, thresh1);
        Core.inRange(mat, red2_lowHSV, red2_highHSV, thresh2);
        Mat thresh = new Mat();
        Core.bitwise_or(thresh1, thresh2, thresh);

        thresh1.release();
        thresh2.release();

        return thresh;
    }

    private Mat blueThresh() {
        Scalar blue_lowHSV = new Scalar (110,64,20);
        Scalar blue_highHSV = new Scalar (130,255,255);
        Mat thresh = new Mat();
        Core.inRange(mat, blue_lowHSV, blue_highHSV, thresh);

        return thresh;
    }

    private Mat yellowThresh() {
        // yellow
        Scalar lowHSV = new Scalar (15,64,20);
        Scalar highHSV = new Scalar (30,255,255);
        Mat thresh = new Mat();

        //yellow
        Core.inRange(mat, lowHSV, highHSV, thresh);

        return thresh;
    }

    public double differenceX () {
        return middleX() - (screenWidth/2.0);
    }

    private double middleX () {
        return target.x + (target.width/2.0);
    }
}
