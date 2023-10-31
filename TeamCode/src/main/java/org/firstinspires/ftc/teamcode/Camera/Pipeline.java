package org.firstinspires.ftc.teamcode.Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {
    OpenCvCamera webcam;
    Telemetry telemetry;
    Mat YCbCr = new Mat();
    Mat leftCrop;
    Mat midleCrop;
    Mat rightCrop;
    double leftavgfin;
    double midleavgfin;
    double rightavgfin;
    Mat output = new Mat();
    Scalar rectColor = new Scalar(255, 255, 255);
    Pipeline (OpenCvCamera webcam, Telemetry telemetry){
        this.webcam = webcam;
        this.telemetry = telemetry;
    }
    boolean viewportPaused;

    public void onViewportTapped(){

        viewportPaused = !viewportPaused;

        if(viewportPaused)
        {
            webcam.pauseViewport();
        }
        else
        {
            webcam.resumeViewport();
        }
    }
    public Telemetry tel1 (Telemetry tel1, Scalar lefavg, Scalar midleavg){
        tel1.addData("leftavgfin",lefavg.val[0]);
        tel1.addData("midleavgfin",midleavg.val[0]);
        return tel1;
    }
    @Override
    public Mat processFrame(Mat input) {
        return input;
    }

}
