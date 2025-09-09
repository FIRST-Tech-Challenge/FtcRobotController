package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionPipelines.WebcamPipelineNotes;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class WebcamInitNotes extends OpMode
{

    // why does Android studio have a spell checker
    // webcamdude is the webcam. I'm not naming it anything else
    OpenCvWebcam webcamdude = null;

    @Override // This stuff happens when you click the init button
    public void init()
    {
        WebcamName webcamname = hardwareMap.get(WebcamName.class, "Webcam 1");
        // Acquire the camera ID
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName() );
        //set the cam name and id to the webcam.
        webcamdude = OpenCvCameraFactory.getInstance().createWebcam(webcamname, cameraMonitorViewId);

        //set webcam Pipeline
        webcamdude.setPipeline(new WebcamPipelineNotes());

        //You're creating an instance of the AsyncCameraOpenListener class. The class contains the two methods, onOpened and onError, which you are overriding with your code. That instance is passed to the openCameraDeviceAsync method as a parameter.
        webcamdude.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcamdude.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Webcam not working");
            }
        });
    }

    @Override // This stuff happens when you click the play button
    public void loop()
    {
        telemetry.addLine("Why did you click this lmao");
    }
}

