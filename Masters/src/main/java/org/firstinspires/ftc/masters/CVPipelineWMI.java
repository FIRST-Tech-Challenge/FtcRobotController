package org.firstinspires.ftc.masters;

import static org.opencv.imgproc.Imgproc.GaussianBlur;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CVPipelineWMI extends OpenCvPipeline {

    private final Rect interest = new Rect(304, 231, 32, 50);
    Telemetry telemetry;
    private final TelemetryPacket packet;

    public CVPipelineWMI(Telemetry telemetry, TelemetryPacket packet) {
        this.telemetry = telemetry;
        this.packet = packet;
    }

    Mat LAB = new Mat(), dst = new Mat();
    Mat A = new Mat();
    Mat B = new Mat();
    Mat region_a = new Mat();
    Mat region_b = new Mat();

    int avg_a = 0;
    int avg_b = 0;

    int detected;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    void inputToLAB(Mat input) {

        Imgproc.cvtColor(input, LAB, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(LAB, A, 0);
        Core.extractChannel(LAB, B, 1);
    }

    @Override
    public Mat processFrame(Mat input) {

        GaussianBlur( input, dst, new Size(39, 39), 0, 0 );

        inputToLAB(dst);


        region_a = A.submat(interest);
        region_b = B.submat(interest);

        avg_a = (int) Core.mean(region_a).val[0];
        avg_b = (int) Core.mean(region_b).val[0];

        packet.put("avg a ", avg_a);
        packet.put("avg b ", avg_b);

        if (avg_a >= 78) {
            detected = 1;// 1
        } else if (avg_b >= 15) {
            detected = 2;// 2
        } else {
            detected = 3;// 3
        }

        packet.put("detected", detected);

        dashboard.sendTelemetryPacket(packet);

        telemetry.update();

        Imgproc.rectangle(dst, new Point(interest.x, interest.y), new Point(interest.x + interest.width, interest.y+ interest.height), new Scalar(0,255,0),1 );

        return dst;
    }
}