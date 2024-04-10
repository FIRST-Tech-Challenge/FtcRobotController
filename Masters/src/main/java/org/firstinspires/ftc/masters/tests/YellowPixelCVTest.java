package org.firstinspires.ftc.masters.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class YellowPixelCVTest extends OpenCvPipeline {

    public final Rect interestMid = new Rect(210, 15, 70, 30);
    public final Rect interestRight = new Rect(360, 15, 70, 30);
    private final Rect pix = new Rect(1,1,1,1);

    public enum pos {
        LEFT,
        MID,
        RIGHT,
    }

    public pos position = pos.LEFT;

    Telemetry telemetry;
    private final TelemetryPacket packet;

    public YellowPixelCVTest(Telemetry telemetry, TelemetryPacket packet) {
        this.telemetry = telemetry;
        this.packet = packet;
    }

    Mat HSV = new Mat();
    Mat H = new Mat();
    Mat S = new Mat();
    Mat V = new Mat();

    Mat region_h_mid = new Mat();
    Mat region_s_mid = new Mat();
    Mat region_h_right = new Mat();
    Mat region_s_right = new Mat();

    int avg_h_mid = 0;
    int avg_s_mid = 0;
    int avg_h_right = 0;
    int avg_s_right = 0;

    Mat mask = new Mat(), diff_im = new Mat();

    FtcDashboard dashboard = FtcDashboard.getInstance();

    void inputToHSV(Mat input) {
        Imgproc.cvtColor(input,HSV,Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(HSV, H, 0);
        Core.extractChannel(HSV, S, 1);
        Core.extractChannel(HSV, V, 2);
    }

    @Override
    public Mat processFrame(Mat input) {

        inputToHSV(input);

        Core.inRange(HSV, new Scalar(0,75,0), new Scalar(255,255,255), mask);

        diff_im = new Mat();
        Core.add(diff_im, Scalar.all(0), diff_im);
        Core.bitwise_not(mask,mask);
        input.copyTo(diff_im, mask);
        diff_im.copyTo(input);
        diff_im.release();

        inputToHSV(input);

        region_h_mid = H.submat(interestMid);
        region_s_mid = S.submat(interestMid);
        region_h_right = H.submat(interestRight);
        region_s_right = S.submat(interestRight);

        avg_h_mid = (int) Core.mean(region_h_mid).val[0];
        avg_s_mid = (int) Core.mean(region_s_mid).val[0];
        avg_h_right = (int) Core.mean(region_h_right).val[0];
        avg_s_right = (int) Core.mean(region_s_right).val[0];

        if (avg_s_mid <avg_s_right) {
            position = pos.MID;
        } else {
            position = pos.RIGHT;
        }

        telemetry.addData("position", position);
        telemetry.update();

        Imgproc.rectangle(input, new Point(interestMid.x, interestMid.y), new Point(interestMid.x + interestMid.width, interestMid.y+ interestMid.height), new Scalar(0,255,0),1 );
        Imgproc.rectangle(input, new Point(interestRight.x, interestRight.y), new Point(interestRight.x + interestRight.width, interestRight.y+ interestRight.height), new Scalar(0,255,0),1 );

        packet.put("Pos:",position);
        packet.put("avg left",avg_s_mid);
        packet.put("avg right",avg_s_right);

        dashboard.sendTelemetryPacket(packet);

        return input; //dst
    }
}