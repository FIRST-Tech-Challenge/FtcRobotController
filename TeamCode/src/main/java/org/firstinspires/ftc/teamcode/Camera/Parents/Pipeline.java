package org.firstinspires.ftc.teamcode.Camera.Parents;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {
    protected OpenCvCamera webcam;
    protected Telemetry telemetry;
    protected Mat YCbCr = new Mat();
    protected Mat leftCrop;
    protected Mat midleCrop;
    protected Mat rightCrop;
    protected double leftavgfin;
    protected double midleavgfin;
    protected double rightavgfin;
    protected  Mat output = new Mat();
    protected Scalar rectColor = new Scalar(255, 255, 255);
    protected Pipeline (OpenCvCamera webcam, Telemetry telemetry){
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
    protected Telemetry tel1 (Telemetry tel1, Scalar lefavg, Scalar midleavg){
        tel1.addData("leftavgfin",lefavg.val[0]);
        tel1.addData("midleavgfin",midleavg.val[0]);
        return tel1;
    }
    @Override
    public Mat processFrame(Mat input) {
        return input;
    }

}
