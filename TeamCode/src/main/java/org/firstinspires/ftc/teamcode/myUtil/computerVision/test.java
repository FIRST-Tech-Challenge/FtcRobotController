package org.firstinspires.ftc.teamcode.myUtil.computerVision;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.myUtil.Hardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
//@Disabled
//@Config
@TeleOp(name="VisionTest")
public class test extends OpMode {
    public static compVis.Colors color = compVis.Colors.RED;
    OpenCvWebcam webcam1 = null;
    compVis test;
    @Override
    public void init() {
        try{
            WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
            test = new compVis(this, color, new Hardware());
            webcam1.setPipeline(test);
            webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {

                }
            });

        }catch(Exception e){}
    }

    @Override
    public void loop() {
        telemetry.addData("Location",test.loc);
        telemetry.update();

    }

}
