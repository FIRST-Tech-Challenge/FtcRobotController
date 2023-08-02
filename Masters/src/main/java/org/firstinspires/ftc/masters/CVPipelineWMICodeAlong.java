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
import org.openftc.easyopencv.OpenCvPipeline;

public class CVPipelineWMICodeAlong extends OpenCvPipeline {


    Telemetry telemetry;
    private final TelemetryPacket packet;

    public CVPipelineWMICodeAlong(Telemetry telemetry, TelemetryPacket packet) {
        this.telemetry = telemetry;
        this.packet = packet;
    }


    private final Rect roi = new Rect(304, 256, 32, 50);

    Mat LAB = new Mat(), dst = new Mat();
    Mat A = new Mat();
    Mat B = new Mat();
    Mat roi_a = new Mat();
    Mat roi_b = new Mat();

    int avg_roi_a = 0;
    int avg_roi_b = 0;

//    int detected;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    void inputToLAB(Mat input) {

        Imgproc.cvtColor(input, LAB, Imgproc.COLOR_RGB2Lab);
        Core.extractChannel(LAB, A, 1);
        Core.extractChannel(LAB, B, 2);
    }

    @Override
    public Mat processFrame(Mat input) {

//        GaussianBlur( input, dst, new Size(39, 39), 0, 0 );

        inputToLAB(input);

        roi_a = A.submat(roi);
        roi_b = B.submat(roi);

        avg_roi_a = (int) Core.mean(roi_a).val[0];
        avg_roi_b = (int) Core.mean(roi_b).val[0];

        packet.put("avg a ", avg_roi_a);
        packet.put("avg b ", avg_roi_b);

//        if (avg_a >= 9 && avg_b >= 170) {
//            detected = 3;// 3
//        } else if (avg_a >= 9) {
//            detected = 2;// 2
//        } else if (avg_b >= 170) {
//            detected = 1;// 1
//        }

//


//        packet.put("detected", detected);

        dashboard.sendTelemetryPacket(packet);

        telemetry.update();

        Imgproc.rectangle(input, new Point(roi.x, roi.y), new Point(roi.x + roi.width, roi.y+ roi.height), new Scalar(0,255,0),1 );

        return input;
    }
}