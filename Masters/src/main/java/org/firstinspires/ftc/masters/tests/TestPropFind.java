package org.firstinspires.ftc.masters.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TestPropFind extends OpenCvPipeline {

    private final Rect interestMid = new Rect(178, 180, 32, 50);
    private final Rect interestRight = new Rect(460, 230, 32, 50);

    private enum pos {
        LEFT,
        MID,
        RIGHT,
    }

    public pos position = pos.LEFT;

    Telemetry telemetry;
    private final TelemetryPacket packet;

    public TestPropFind(Telemetry telemetry, TelemetryPacket packet) {
        this.telemetry = telemetry;
        this.packet = packet;
    }

    Mat LAB = new Mat(), dst = new Mat();
    Mat A = new Mat();
    Mat B = new Mat();
    Mat region_a_mid = new Mat();
    Mat region_b_mid = new Mat();
    Mat region_a_right = new Mat();
    Mat region_b_right = new Mat();

    int avg_a_mid = 0;
    int avg_b_mid = 0;
    int avg_a_right = 0;
    int avg_b_right = 0;

    Mat mask = new Mat(), diff_im = new Mat();

    int detected;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    void inputToLAB(Mat input) {

        Imgproc.cvtColor(input, LAB, Imgproc.COLOR_RGB2Lab);
        Core.extractChannel(LAB, A, 0);
        Core.extractChannel(LAB, B, 1);
    }

    @Override
    public Mat processFrame(Mat input) {

        inputToLAB(input);

        Core.inRange(LAB, new Scalar(0,0,0), new Scalar(240,150,255), mask); //Black out Red

        diff_im = new Mat();
        Core.add(diff_im, Scalar.all(0), diff_im);

        input.copyTo(diff_im, mask);
        diff_im.copyTo(input);
        diff_im.release();

        Core.extractChannel(LAB, A, 0);

        inputToLAB(input);

        region_a_mid = A.submat(interestMid);
        region_b_mid = B.submat(interestMid);
        region_a_right = A.submat(interestRight);
        region_b_right = B.submat(interestRight);

        avg_a_mid = (int) Core.mean(region_a_mid).val[0];
        avg_b_mid = (int) Core.mean(region_b_mid).val[0];
        avg_a_right = (int) Core.mean(region_a_right).val[0];
        avg_b_right = (int) Core.mean(region_b_right).val[0];

        if (avg_a_mid<5) {
            position = pos.MID;
        } else if (avg_a_right<5) {
            position = pos.RIGHT;
        } else {
            position = pos.LEFT;
        }

        packet.put("avg a mid", avg_a_mid);
        packet.put("avg b mid", avg_b_mid);
        packet.put("avg a right", avg_a_right);
        packet.put("avg b right", avg_b_right);
        packet.put("pos: ", position);

        dashboard.sendTelemetryPacket(packet);
        telemetry.update();

        Imgproc.rectangle(input, new Point(interestMid.x, interestMid.y), new Point(interestMid.x + interestMid.width, interestMid.y+ interestMid.height), new Scalar(0,255,0),1 );
        Imgproc.rectangle(input, new Point(interestRight.x, interestRight.y), new Point(interestRight.x + interestRight.width, interestRight.y+ interestRight.height), new Scalar(0,255,0),1 );


        return input; //dst
    }
}