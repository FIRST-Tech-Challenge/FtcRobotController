package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import java.util.ArrayList;

public class RandomOpenCV extends OpMode {

    static final int STREAM_WIDTH = 1080; // modify for your camera
    static final int STREAM_HEIGHT = 720; // modify for your camera
    OpenCvWebcam webcam;
    SamplePipeline pipeline;

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "WebcamMain"); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed","");
                telemetry.update();
            }
        });

    }

    @Override
    public void loop() {
        telemetry.addData("Image Analysis:",pipeline.getAnalysis());
        telemetry.update();
    }
}

class SampleOfPipeline extends RandomOpenCV {
    Mat mRgba;
    Mat mHsv;

    int avg;
    int avgA;

    static final int WidthRectA = 130;
    static final int HeightRectA = 110;

    static final Point RectATopLeftAnchor = new Point((STREAM_WIDTH - WidthRectA) / 2 + 300, ((STREAM_HEIGHT - HeightRectA) / 2) - 100);
    Point RectATLCorner = new Point(
            RectATopLeftAnchor.x,
            RectATopLeftAnchor.y);
    Point RectABRCorner = new Point(
            RectATopLeftAnchor.x + WidthRectA,
            RectATopLeftAnchor.y + HeightRectA);

    void inputToRGB(Mat input) {
        Imgproc.cvtColor(input, mHsv, Imgproc.COLOR_RGB2HSV);
        Scalar mColorHsv = Core.sumElems(mHsv);
        int pointCount = STREAM_HEIGHT*STREAM_HEIGHT;
        ArrayList<Mat> rgbaChannels = new ArrayList<Mat>(3);
        Core.split(mRgba, rgbaChannels);

        for (int i = 0; i < mColorHsv.val.length; i++) {
            mColorHsv.val[i] /= pointCount;
        }

        Scalar mColorRgb = convertScalarHsv2Rgba(mColorHsv);

        int R = (int) mColorRgb.val[0];
        int G = (int) mColorRgb.val[1];
        int B = (int) mColorRgb.val[2];

    }



    public int getAnalysis() {
        return avg;
    }
    public int getRectA_Analysis() {
        return avgA;
    }

    private Scalar convertScalarHsv2Rgba(Scalar hsvColor) {
        Mat pointMatRgba = new Mat();
        Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
        Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB);

        return new Scalar(pointMatRgba.get(0, 0));
    }
}
