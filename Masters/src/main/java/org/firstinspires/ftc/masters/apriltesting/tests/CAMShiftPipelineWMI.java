package org.firstinspires.ftc.masters.apriltesting.tests;

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

public class CAMShiftPipelineWMI extends OpenCvPipeline {

    Mat hsv_roi = new Mat();
    Mat mask = new Mat();
    Mat input2 = new Mat();

    // hardcode the initial location of window
    private final Rect trackWindow = new Rect(18, 15, 36, 30);
    private final Rect interest = new Rect(658, 574, 72, 60);
    Telemetry telemetry;
    private final TelemetryPacket packet;

    public CAMShiftPipelineWMI(Telemetry telemetry, TelemetryPacket packet) {
        this.telemetry = telemetry;
        this.packet = packet;
    }

    public Point center;
    public Size size;

    TermCriteria term_crit;

    RotatedRect rot_rect;

    Mat region_a;
    Mat region_b;

    Mat roi, roi_hist, hsv, dst;
    MatOfFloat range = new MatOfFloat(0, 256);
    MatOfInt histSize = new MatOfInt(180);
    MatOfInt channels = new MatOfInt(0);

    Point[] points;

    Mat LAB = new Mat();
    Mat A = new Mat();
    Mat B = new Mat();


    int avg_a = 0;
    int avg_b = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    void inputToLAB(Mat input) {

        Imgproc.cvtColor(input, LAB, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(LAB, A, 1);
        Core.extractChannel(LAB, B, 2);
    }

    @Override
    public Mat processFrame(Mat input) {

        input2 = input.submat(interest);
        roi = input2.submat(trackWindow);
        Imgproc.cvtColor(roi, hsv_roi, Imgproc.COLOR_BGR2HSV);
        Core.inRange(hsv_roi, new Scalar(0, 60, 32), new Scalar(180, 255, 150), mask);

        roi_hist = new Mat();
        Imgproc.calcHist(Collections.singletonList(hsv_roi), channels, mask, roi_hist, histSize, range);
        Core.normalize(roi_hist, roi_hist, 0, 255, Core.NORM_MINMAX);

        term_crit = new TermCriteria(TermCriteria.EPS | TermCriteria.COUNT, 100, .1);

        hsv = new Mat();
        dst = new Mat();
        Imgproc.cvtColor(input2, hsv, Imgproc.COLOR_BGR2HSV);
        Imgproc.calcBackProject(Collections.singletonList(hsv), channels, roi_hist, dst, range, 1);

        rot_rect = Video.CamShift(dst, trackWindow, term_crit);

        points = new Point[4];
        rot_rect.points(points);
        for (int i = 0; i < 4 ;i++) {
            Imgproc.line(input, new Point(points[i].x+interest.x,points[i].y+interest.y), new Point(points[(i+1)%4].x+interest.x,points[(i+1)%4].y+interest.y), new Scalar(0, 255, 0),1);
        }

        Imgproc.rectangle(input, new Point(interest.x, interest.y), new Point(interest.x + interest.width, interest.y+ interest.height), new Scalar(255,255,255),1 );

        telemetry.addData("rot_rect: ", rot_rect.toString());
        packet.put("rot_rect: ", rot_rect.toString());

        center = rot_rect.center;
        size = rot_rect.size;

        inputToLAB(input);

        region_a = A.submat(interest);
        region_b = B.submat(interest);

        avg_a = (int) Core.mean(region_a).val[0];
        avg_b = (int) Core.mean(region_b).val[0];

        packet.put("avg a ", avg_a);
        packet.put("avg b ", avg_b);

        dashboard.sendTelemetryPacket(packet);

        telemetry.update();

        return input;
    }
}