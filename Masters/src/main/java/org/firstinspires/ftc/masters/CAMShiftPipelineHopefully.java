package org.firstinspires.ftc.masters;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.Video;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Collections;

public class CAMShiftPipelineHopefully extends OpenCvPipeline {

    public String data;


    Mat hsv_roi = new Mat();
    Mat mask = new Mat();

    // hardcode the initial location of window
    private final Rect trackWindow = new Rect(150, 60, 63, 125);
    Telemetry telemetry;

    public CAMShiftPipelineHopefully(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {

        Mat roi = input.submat(trackWindow);
        Imgproc.cvtColor(roi, hsv_roi, Imgproc.COLOR_BGR2HSV);
        Core.inRange(hsv_roi, new Scalar(0, 60, 32), new Scalar(180, 255, 255), mask);

        MatOfFloat range = new MatOfFloat(0, 256);
        Mat roi_hist = new Mat();
        MatOfInt histSize = new MatOfInt(180);
        MatOfInt channels = new MatOfInt(0);
        Imgproc.calcHist(Collections.singletonList(hsv_roi), channels, mask, roi_hist, histSize, range);
        Core.normalize(roi_hist, roi_hist, 0, 255, Core.NORM_MINMAX);

        TermCriteria term_crit = new TermCriteria(TermCriteria.EPS | TermCriteria.COUNT, 100, .1);


        Mat hsv = new Mat() , dst = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
        Imgproc.calcBackProject(Collections.singletonList(hsv), channels, roi_hist, dst, range, 1);


        RotatedRect rot_rect = Video.CamShift(dst, trackWindow, term_crit);


        Point[] points = new Point[4];
        rot_rect.points(points);
        for (int i = 0; i < 4 ;i++) {
            Imgproc.line(input, points[i], points[(i+1)%4], new Scalar(255, 0, 0),2);
        }
        data = rot_rect.toString();
        telemetry.addData("rot_rect: ", rot_rect.toString());
        telemetry.update();
        return input;
    }
}