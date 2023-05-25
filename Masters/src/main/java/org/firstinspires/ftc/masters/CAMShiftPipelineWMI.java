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

public class CAMShiftPipelineWMI extends OpenCvPipeline {

    Mat hsv_roi = new Mat();
    Mat mask = new Mat();
    Mat input2 = new Mat();

    // hardcode the initial location of window
    private final Rect trackWindow = new Rect(18, 40, 36, 80);
    private final Rect interest = new Rect(658, 534, 72, 160);
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

    Mat roi, roi_hist, hsv, dst;
    MatOfFloat range = new MatOfFloat(0, 256);
    MatOfInt histSize = new MatOfInt(180);
    MatOfInt channels = new MatOfInt(0);

    Point[] points;

    FtcDashboard dashboard = FtcDashboard.getInstance();

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
            Imgproc.line(input, new Point(points[i].x+658,points[i].y+534), new Point(points[(i+1)%4].x+658,points[(i+1)%4].y+534), new Scalar(0, 255, 0),1);
        }

        Imgproc.rectangle(input, new Point(interest.x, interest.y), new Point(interest.x + interest.width, interest.y+ interest.height), new Scalar(255,255,255),1 );

        telemetry.addData("rot_rect: ", rot_rect.toString());
        packet.put("rot_rect: ", rot_rect.toString());

        center = rot_rect.center;
        size = rot_rect.size;

        dashboard.sendTelemetryPacket(packet);

        telemetry.update();

        return input;
    }
}