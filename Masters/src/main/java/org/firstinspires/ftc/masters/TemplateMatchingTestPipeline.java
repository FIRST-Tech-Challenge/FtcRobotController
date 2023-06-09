package org.firstinspires.ftc.masters;

import static org.opencv.imgcodecs.Imgcodecs.imread;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvPipeline;

public class TemplateMatchingTestPipeline extends OpenCvPipeline {

    private final Rect interest = new Rect(329, 237, 36, 60);

    Telemetry telemetry;
    private final TelemetryPacket packet;

    Mat templ1, templ2, templ3;

    public TemplateMatchingTestPipeline(Telemetry telemetry, TelemetryPacket packet) {
        this.telemetry = telemetry;
        this.packet = packet;

        templ1 = imread("C:\\Users\\ollie\\Documents\\GitHub\\UltimateGoal\\Masters\\src\\main\\java\\org\\firstinspires\\ftc\\masters\\signal_sleeve_side_2.png");
        templ2 = imread("C:\\Users\\ollie\\Documents\\GitHub\\UltimateGoal\\Masters\\src\\main\\java\\org\\firstinspires\\ftc\\masters\\signal_sleeve_side_2.png");
        templ3 = imread("C:\\Users\\ollie\\Documents\\GitHub\\UltimateGoal\\Masters\\src\\main\\java\\org\\firstinspires\\ftc\\masters\\signal_sleeve_side_2.png");
    }

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public Mat processFrame(Mat input) {


        templ2 = imread("C:\\Users\\ollie\\Documents\\GitHub\\UltimateGoal\\Masters\\src\\main\\java\\org\\firstinspires\\ftc\\masters\\signal_sleeve_side_2.png");

//        Imgproc.rectangle(input, new Point(interest.x, interest.y), new Point(interest.x + interest.width, interest.y+ interest.height), new Scalar(0,255,0),1 );
//
//        Mat result1 = new Mat();
//        Mat result2 = new Mat();
//        Mat result3 = new Mat();
//        Mat img_display = new Mat();
//        input.copyTo(img_display);
//        int result_cols = input.cols() - templ1.cols() + 1;
//        int result_rows = input.rows() - templ1.rows() + 1;
//
//        result1.create(result_rows, result_cols, CvType.CV_32FC1);
//        result2.create(result_rows, result_cols, CvType.CV_32FC1);
//        result3.create(result_rows, result_cols, CvType.CV_32FC1);
//
//        Imgproc.matchTemplate(input, templ1, result1, Imgproc.TM_CCORR_NORMED);
//        Imgproc.matchTemplate(input, templ2, result2, Imgproc.TM_CCORR_NORMED);
//        Imgproc.matchTemplate(input, templ3, result3, Imgproc.TM_CCORR_NORMED);
//
//        Core.normalize(result1, result1, 0, 1, Core.NORM_MINMAX, -1, new Mat());
//        Core.normalize(result2, result2, 0, 1, Core.NORM_MINMAX, -1, new Mat());
//        Core.normalize(result3, result3, 0, 1, Core.NORM_MINMAX, -1, new Mat());
//
//        Point matchLoc1;
//        Point matchLoc2;
//        Point matchLoc3;
//
//        Core.MinMaxLocResult mmr1 = Core.minMaxLoc(result1);
//        Core.MinMaxLocResult mmr2 = Core.minMaxLoc(result2);
//        Core.MinMaxLocResult mmr3 = Core.minMaxLoc(result3);
//
//        matchLoc1 = mmr1.maxLoc;
//        matchLoc2 = mmr2.maxLoc;
//        matchLoc3 = mmr3.maxLoc;
//
//        Imgproc.rectangle(img_display, matchLoc1, new Point(matchLoc1.x + templ1.cols(), matchLoc1.y + templ1.rows()),
//                new Scalar(0, 0, 0), 2, 8, 0);
//        Imgproc.rectangle(img_display, matchLoc2, new Point(matchLoc2.x + templ2.cols(), matchLoc2.y + templ2.rows()),
//                new Scalar(0, 0, 0), 2, 8, 0);
//        Imgproc.rectangle(img_display, matchLoc3, new Point(matchLoc3.x + templ3.cols(), matchLoc3.y + templ3.rows()),
//                new Scalar(0, 0, 0), 2, 8, 0);
//        Imgproc.rectangle(result1, matchLoc1, new Point(matchLoc1.x + templ1.cols(), matchLoc1.y + templ1.rows()),
//                new Scalar(0, 0, 0), 2, 8, 0);
//        Imgproc.rectangle(result2, matchLoc2, new Point(matchLoc2.x + templ2.cols(), matchLoc2.y + templ2.rows()),
//                new Scalar(0, 0, 0), 2, 8, 0);
//        Imgproc.rectangle(result3, matchLoc3, new Point(matchLoc3.x + templ3.cols(), matchLoc3.y + templ3.rows()),
//                new Scalar(0, 0, 0), 2, 8, 0);
//
//
//        result1.convertTo(result1, CvType.CV_8UC1, 255.0);
//        result2.convertTo(result2, CvType.CV_8UC1, 255.0);
//        result3.convertTo(result3, CvType.CV_8UC1, 255.0);
//
//        packet.put("yup1",result1.get((int)matchLoc1.x,(int)matchLoc1.y));
//        packet.put("yup2",result2.get((int)matchLoc2.x,(int)matchLoc2.y));
//        packet.put("yup3",result3.get((int)matchLoc3.x,(int)matchLoc3.y));
//        dashboard.sendTelemetryPacket(packet);
//
//        result1.release(); result2.release(); result3.release();

        return templ2;
    }
}