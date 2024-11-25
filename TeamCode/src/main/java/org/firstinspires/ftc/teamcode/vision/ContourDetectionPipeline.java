package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ContourDetectionPipeline extends OpenCvPipeline
{
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private final Telemetry tele = dash.getTelemetry();

    public enum processors { OFF, SIMPLE, COMPLEX, SIMPLER }
    private processors processorSetting = processors.COMPLEX; // default to no processing as not to slow down imps

    @Override
    public Mat processFrame(Mat input) {
        if(processorSetting == processors.COMPLEX){
            // will be to locate blocks
            Mat gray = new Mat(input.rows(), input.cols(), input.type());
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
            Mat binary = new Mat(input.rows(), input.cols(), input.type(), new Scalar(0));
            Imgproc.threshold(gray, binary, 100, 255, Imgproc.THRESH_BINARY_INV);
            //Finding Contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchey = new Mat();
            Imgproc.findContours(binary, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            //Drawing the Contours
            Scalar color = new Scalar(0, 0, 255);
            Imgproc.drawContours(input, contours, -1, color, 2, Imgproc.LINE_8, hierarchey, 2, new Point() );

        } else if(processorSetting == processors.SIMPLER){
            // super simple thing
        } else if(processorSetting == processors.SIMPLE) {
            // simple thing
        } else if (processorSetting == processors.OFF){
            // do something when not processing? idk
        }

        FtcDashboard.getInstance().getTelemetry().update();
        return input;
    }
}