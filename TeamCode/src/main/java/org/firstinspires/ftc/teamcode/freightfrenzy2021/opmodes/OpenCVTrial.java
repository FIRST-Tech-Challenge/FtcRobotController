package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsColorSensor;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.opencvpipelines.BarCodeScanner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous
public class OpenCVTrial extends LinearOpMode {

    String logTag = "EBOTS";


    BarCodeScanner barCodeScanner;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        barCodeScanner = new BarCodeScanner();
//        rightSidePipeline = new RightSidePipeline();
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        // With live preview
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        EbotsColorSensor bucketColorSensor = new EbotsColorSensor(hardwareMap);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                Log.d(logTag, "The camera is now open...");
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(barCodeScanner);

            }
            @Override
            public void onError(int errorCode)
            {
                Log.d(logTag, "There was an error");
            }
        });

        while(!isStarted() && !isStopRequested()){
            // just stream here
            telemetry.update();
        }

        waitForStart();
        camera.stopStreaming();
        //end it

    }

    private String getColor(int hue){
        String color = "unknown";

        if (hue < 45 | hue > 150){
            color = "Red";
        } else if (hue > 60 && hue < 85){
            color = "Green";
        }
        return color;
    }
}
