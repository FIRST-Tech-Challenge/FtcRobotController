package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.vision.ColorDetect;
import org.firstinspires.ftc.teamcode.vision.DetectedColor;
import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import java.util.concurrent.TimeUnit;


/** Subsystem */
public class ClawCamera extends SubsystemBase {

    // Used for managing the color detection process.
    private ColorDetect myColorDetectProcessor;

    // Local objects and variables here
    private final VisionPortal CameraPortal;

    /** Place code here to initialize subsystem */
    public ClawCamera(String cameraName) {

        CameraPortal = new VisionPortal.Builder()
                .setCamera(RobotContainer.ActiveOpMode.hardwareMap.get(WebcamName.class, cameraName))
                .setCameraResolution(new Size(640, 480))
                //.setCameraResolution(new Size(1280,720)) if have an HD camera
                .addProcessor(myColorDetectProcessor)
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        // set camera exposure and gain
        // values used from example code
//        setCameraExposure(2, 250);

        RobotContainer.DashBoard.startCameraStream(CameraPortal, 0);
    }

    // Updates dashboard with robot odometry info
    public void updateDashboard() {
        // Get the list of detected colors
        List<DetectedColor> detectedColors = GetCurrentColorDetections();

        // First, clear the previous data from the dashboard (if needed)
        RobotContainer.DBTelemetry.clear(); // Optional, depending on your telemetry system

        // Add information for each detected color
        for (int i = 0; i < detectedColors.size(); i++) {
            DetectedColor detected = detectedColors.get(i);

            // Add detected color name and area to the telemetry
            RobotContainer.DBTelemetry.addData("Detected Color " + (i + 1), detected.colorName);
            RobotContainer.DBTelemetry.addData("Color " + (i + 1) + " Area", detected.area);

            // Add bounding box information (for instance, top-left and bottom-right coordinates)
            RobotContainer.DBTelemetry.addData("Color " + (i + 1) + " Bounding Box",
                    "Top-left: (" + detected.boundingBox.x + ", " + detected.boundingBox.y + "), " +
                            "Bottom-right: (" + (detected.boundingBox.x + detected.boundingBox.width) + ", " +
                            (detected.boundingBox.y + detected.boundingBox.height) + ")"
            );
        }

        // Update the telemetry to reflect the latest data on the dashboard
        RobotContainer.DBTelemetry.update();
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
        updateDashboard();
    }

    // get current AprilTag detections (if any) from camera
    // returns list containing info on each tag detected
    public List<DetectedColor> GetCurrentColorDetections() {
        return myColorDetectProcessor.getDetectedColors();
    }

    // get camera frames per second
    public double GetCameraFPS () {
        return CameraPortal.getFps();
    }

    // use to turn on/off AprilTag processing
    public void EnableColorDetectProcessing (boolean enable) {
        CameraPortal.setProcessorEnabled(myColorDetectProcessor, enable);
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