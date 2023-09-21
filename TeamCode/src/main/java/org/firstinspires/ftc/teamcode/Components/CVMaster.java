package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.dashboard;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RFAprilCam;
import org.firstinspires.ftc.teamcode.Components.CV.Pipelines.SpikeObserverPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * Warren
 * Class that contaions all CV processes, should make it easier to use
 */
public class CVMaster {
    private OpenCvWebcam webcam;
    private RFAprilCam cam;
    private SpikeObserverPipeline openSleevi = null;

    private boolean isObservingPole = false,isObservingCone=false;

    private boolean isStreaming = false;

    /**
     * initializes opencv webcam, starts observing spike, logs that opencv webcam is initialized and recording spike pipeline to general surface log
     */
    public CVMaster() {
        webcam = OpenCvCameraFactory.getInstance().createWebcam(op.hardwareMap.get(WebcamName.class, "Webcam1"));
        observeSpike();
    }

    /**
     * observes spike, logs that spike is being observed general surface log
     */
    public void observeSpike() {
        openSleevi = new SpikeObserverPipeline();
        webcam.setPipeline(openSleevi);
        isObservingPole = true;
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.setPipeline(openSleevi);
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                dashboard.startCameraStream(webcam, 10);

            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        isStreaming = true;
    }

    /**
     * gets calculated position from spike pipeline
     * @return position from spike pipeline
     */
    public int getPosition() {
        return openSleevi.getPosition();
    }

    /**
     * switches to apriltag camera, logs to general surface log
     */
    public void switchToApril() {
        webcam.stopRecordingPipeline();
        cam = new RFAprilCam();
        isStreaming=false;
    }

    /**
     * updates the aprilTag info if you arre currently on aprilTag mode, logs to genereal surface(inside RFAprilCam class)
     */
    public void update(){
        if(!isStreaming){
            cam.update();
        }
    }
}
