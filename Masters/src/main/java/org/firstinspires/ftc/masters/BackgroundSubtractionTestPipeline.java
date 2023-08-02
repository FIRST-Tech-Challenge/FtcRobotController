package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.BackgroundSubtractor;
import org.opencv.video.Video;
import org.openftc.easyopencv.OpenCvPipeline;

public class BackgroundSubtractionTestPipeline extends OpenCvPipeline {

    Telemetry telemetry;
    private final TelemetryPacket packet;

    public BackgroundSubtractionTestPipeline(Telemetry telemetry, TelemetryPacket packet) {
        this.telemetry = telemetry;
        this.packet = packet;
    }

    Mat LAB = new Mat();
    Mat A = new Mat();
    Mat B = new Mat();

    int avg_a = 0;
    int avg_b = 0;

    private final Rect interest = new Rect(329, 237, 36, 60);

    Mat region_a;
    Mat region_b;

    BackgroundSubtractor backSub = Video.createBackgroundSubtractorKNN();
    Mat fgMask = new Mat(), diff_im = new Mat();

    FtcDashboard dashboard = FtcDashboard.getInstance();

    int detected = 0;

    void inputToLAB(Mat input) {

        Imgproc.cvtColor(input, LAB, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(LAB, A, 1);
        Core.extractChannel(LAB, B, 2);
    }

    @Override
    public Mat processFrame(Mat input) {

        backSub.apply(input, fgMask);

        diff_im = new Mat();
        Core.add(diff_im, Scalar.all(0), diff_im);

        input.copyTo(diff_im, fgMask);
        diff_im.copyTo(input);
        diff_im.release();

        Imgproc.rectangle(input, new Point(interest.x, interest.y), new Point(interest.x + interest.width, interest.y+ interest.height), new Scalar(0,255,0),1 );

        inputToLAB(input);

        region_a = A.submat(interest);
        region_b = B.submat(interest);

        avg_a = (int) Core.mean(region_a).val[0];
        avg_b = (int) Core.mean(region_b).val[0];

        packet.put("avg a ", avg_a);
        packet.put("avg b ", avg_b);

        dashboard.sendTelemetryPacket(packet);

        if (avg_a > 60) {
            detected = 2;
        } else if (avg_a < 35) {
            detected = 1;
        } else {
            detected = 3;
        }

        packet.put("detection", detected);


        return input;
    }
}