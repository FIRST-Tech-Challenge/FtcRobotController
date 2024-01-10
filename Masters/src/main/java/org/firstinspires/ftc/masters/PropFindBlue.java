package org.firstinspires.ftc.masters;

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
public class PropFindBlue extends OpenCvPipeline {

    public final Rect interestMid = new Rect(217, 48, 32, 50);
    public final Rect interestRight = new Rect(530, 90, 32, 50);
    private final Rect pix = new Rect(1,1,1,1);

    private enum pos {
        LEFT,
        MID,
        RIGHT,
    }

    public pos position = pos.LEFT;

    Telemetry telemetry;
    private final TelemetryPacket packet;

    public PropFindBlue(Telemetry telemetry, TelemetryPacket packet) {
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

        Core.inRange(HSV, new Scalar(105,40,0), new Scalar(115,255,255), mask);

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

        if (avg_s_mid <5) {
            position = pos.MID;
        } else if (avg_s_right <5) {
            position = pos.RIGHT;
        } else {
            position = pos.LEFT;
        }

        packet.put("avg h mid", avg_h_mid);
        packet.put("avg s mid", avg_s_mid);
        packet.put("avg h right", avg_h_right);
        packet.put("avg s right", avg_s_right);
        packet.put("pos: ", position);
        packet.put("sf",Core.mean(HSV.submat(pix)).val[0]);
        packet.put("sf1",Core.mean(HSV.submat(pix)).val[1]);
        packet.put("sf2",Core.mean(HSV.submat(pix)).val[2]);

        dashboard.sendTelemetryPacket(packet);
        telemetry.update();

        Imgproc.rectangle(input, new Point(interestMid.x, interestMid.y), new Point(interestMid.x + interestMid.width, interestMid.y+ interestMid.height), new Scalar(0,255,0),1 );
        Imgproc.rectangle(input, new Point(interestRight.x, interestRight.y), new Point(interestRight.x + interestRight.width, interestRight.y+ interestRight.height), new Scalar(0,255,0),1 );


        return input; //dst
    }
}