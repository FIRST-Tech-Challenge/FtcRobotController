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

public class VisualOdometryPipelinePowerPlay extends OpenCvPipeline {

    Mat hsv_roi = new Mat();
    Mat mask = new Mat();

    // hardcode the initial location of window
    private final Rect trackWindow = new Rect(150, 60, 63, 125);
    Telemetry telemetry;
    private final TelemetryPacket packet;

    public VisualOdometryPipelinePowerPlay(Telemetry telemetry, TelemetryPacket packet) {
        this.telemetry = telemetry;
        this.packet = packet;
    }

    Mat region_a;
    Mat region_b;

    Mat LAB = new Mat();
    Mat A = new Mat();
    Mat B = new Mat();

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

        telemetry.addData("rot_rect: ", rot_rect.toString());
        packet.put("rot_rect: ", rot_rect.toString());

        center = rot_rect.center;
        size = rot_rect.size;
        double angle = rot_rect.angle;

        inputToLAB(input);


        dashboard.sendTelemetryPacket(packet);


        telemetry.update();

        return input;
    }
}