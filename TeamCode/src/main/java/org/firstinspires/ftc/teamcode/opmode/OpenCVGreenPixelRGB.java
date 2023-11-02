package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipeline.GripPipelineGreenPixelRGB;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name="Green Pixel")
//@Disabled
public class OpenCVGreenPixelRGB extends OpMode {
    static final int STREAM_WIDTH = 1280; // modify for your camera
    static final int STREAM_HEIGHT = 960; // modify for your camera
    OpenCvWebcam webcam;
    GripPipelineGreenPixelRGB pipeline;
    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "TinyCam"); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new GripPipelineGreenPixelRGB();
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
    public void init_loop(){
        Point avgLoc = pipeline.avgContourCoord();
        telemetry.addData("AvgContour.x",avgLoc.x);
        telemetry.addData("AvgContour.y",avgLoc.y);
        telemetry.update();
    }

    @Override
    public void loop() {
       // telemetry.addData("Image Analysis:",pipeline.getRectA_Analysis());
        //telemetry.addData("working?","");
        Point avgLoc = pipeline.avgContourCoord();
        telemetry.addData("AvgContour.x",avgLoc.x);
        telemetry.addData("AvgContour.y",avgLoc.y);

        telemetry.update();
    }

}
