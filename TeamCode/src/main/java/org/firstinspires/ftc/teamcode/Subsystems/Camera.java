package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Size;
import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import java.util.concurrent.TimeUnit;


/** Subsystem */
public class Camera extends SubsystemBase {

    // Used for managing the AprilTag detection process.
    private final AprilTagProcessor myAprilTagProcessor;

    // Local objects and variables here
    private final VisionPortal CameraPortal;

    /** Place code here to initialize subsystem */
    public Camera(String cameraName) {

        // Build the AprilTag processor
        // set parameters of AprilTagProcessor, then use Builder to build
        myAprilTagProcessor = new AprilTagProcessor.Builder()
                //.setTagLibrary(myAprilTagLibrary)
                //.setNumThreads(tbd)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        // set apriltag resolution decimation factor
        SetDecimation(2);

        CameraPortal = new VisionPortal.Builder()
                .setCamera(RobotContainer.ActiveOpMode.hardwareMap.get(WebcamName.class, cameraName))
                .setCameraResolution(new Size(640, 480))
                //.setCameraResolution(new Size(1280,720)) if have an HD camera
                .addProcessor(myAprilTagProcessor)
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.

        // set camera exposure and gain
        // values used from example code
        setCameraExposure(2, 250);

        RobotContainer.DashBoard.startCameraStream(CameraPortal, 0);
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }

    // get fresh AprilTag detections (if any) from camera
    // returns list containing info on each tag detected
    public List<AprilTagDetection> GetFreshAprilTagDetections() {
        return myAprilTagProcessor.getFreshDetections();
    }

    // get current AprilTag detections (if any) from camera
    // returns list containing info on each tag detected
    public List<AprilTagDetection> GetCurrentAprilTagDetections() {
        return myAprilTagProcessor.getDetections();
    }

    // sets decimation of AprilTag processing
    public void SetDecimation (int num) {
        myAprilTagProcessor.setDecimation(num);
    }

    // get camera frames per second
    public double GetCameraFPS () {
        return CameraPortal.getFps();
    }

    // use to turn on/off AprilTag processing
    public void EnableAprilTagProcessing (boolean enable) {
        CameraPortal.setProcessorEnabled(myAprilTagProcessor, enable);
    }

    /** Set the camera gain and exposure. */
    public void setCameraExposure(int exposureMS, int gain) {

        // wait until camera in streaming mode
        while (CameraPortal.getCameraState()!= VisionPortal.CameraState.STREAMING)
        {}

        // set exposure control to manual
        ExposureControl exposureControl = CameraPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);

            RobotContainer.ActiveOpMode.sleep(50);
        }

        // set exposure and gain
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        RobotContainer.ActiveOpMode.sleep(20);
        GainControl gainControl = CameraPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        RobotContainer.ActiveOpMode.sleep(20);
    }

    /** Sets the camera exposure to automatic */
    public void SetAutoCameraExposure() {
        ExposureControl exposureControl = CameraPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Auto);
    }

}