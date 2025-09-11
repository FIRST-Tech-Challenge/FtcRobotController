package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

// vision stuff
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import org.firstinspires.ftc.teamcode.Modules.Camera;

/*
 * We have to have two classes because java does not support multiple inheritance
 *
 * This class is the hardware aspect of the camera, the camera class will be used for the pipeline (what processes the feed)
 *
 */
@Autonomous
public class CameraHardware {

    ///////////////////////////////////////////////
    ////                                     /////
    ////              VARIABLES              /////
    ////                                     /////
    //////////////////////////////////////////////

    // this used to be called webcamdude, I'm looking at you Aaron
    OpenCvWebcam webcam = null;

    ///////////////////////////////////////////////
    ////                                     /////
    ////              FUNCTIONS              /////
    ////                                     /////
    //////////////////////////////////////////////

    public CameraHardware(HardwareMap hardwareMap) {

        // get webcam name
        WebcamName webcamname = hardwareMap.get(WebcamName.class, "Webcam 1");

        // get camera ID
        // this is to display the camera feed on the robot controller screen
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // set webcam to the camera name and id
        // can omit cameraMonitorViewId if you don't want to display the live feed
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamname, cameraMonitorViewId);

        // set webcam pipeline
        webcam.setPipeline(new Camera(0.047, 578.272, 578.272, 402.145, 221.506));

        // this is the asyncrounous camera open listener class
        // it has the methods : onOpened and onError
        // we are overriding these methods
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            // starts the webcam when click play (?)
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
//                telemetry.addLine("Webcam started streaming...");
            }

            // called if the camera could not be opened
            @Override
            public void onError(int errorCode) {
//                telemetry.addLine("Welp you screwed up.");
//                telemetry.addLine("Or maybe I\'m stupid.");
//                telemetry.addLine("Or maybe there\'s just an error with the camera.");

            }
        });
    }
}
