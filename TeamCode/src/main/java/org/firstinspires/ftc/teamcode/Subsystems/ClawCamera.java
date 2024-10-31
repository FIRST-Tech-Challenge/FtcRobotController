package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.vision.ColorDetect;
import org.firstinspires.ftc.teamcode.vision.DetectedColor;
import org.firstinspires.ftc.teamcode.vision.ColorAndOrientationDetect;
import org.firstinspires.ftc.teamcode.vision.DetectedColorWithAngle;
import org.firstinspires.ftc.vision.VisionPortal;
import java.util.List;


/** Subsystem */
public class ClawCamera extends SubsystemBase {

    // Used for managing the color detection process.
    private ColorDetect  myColorDetectProcessor;
    private ColorAndOrientationDetect myColorAndOrienDetProcessor;
    // Local objects and variables here
    private final VisionPortal CameraPortal;

    private boolean dashboardInitialized = false;

    /** Place code here to initialize subsystem */
    public ClawCamera(String cameraName) {
        myColorDetectProcessor = new ColorDetect();
        myColorAndOrienDetProcessor = new ColorAndOrientationDetect();

        CameraPortal = new VisionPortal.Builder()
                .setCamera(RobotContainer.ActiveOpMode.hardwareMap.get(WebcamName.class, cameraName))
                .addProcessors(myColorAndOrienDetProcessor) // add all the processors here
                //.setCameraResolution(new Size(640, 480))
                .setCameraResolution(new Size(640,480))
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        RobotContainer.DashBoard.startCameraStream(CameraPortal, 0);

    }
    @Override
    public void periodic() {
//        if (!dashboardInitialized) {
//            initializeDashboard();
//            dashboardInitialized = true;
//        }
//        updateDashboard();
    }

    // Method to initialize dashboard with default (null) values
    private void initializeDashboard() {
        // Set up a fixed number of slots on the dashboard for detected colors
        for (int i = 0; i < 4; i++) { // Assuming a maximum of 10 color detections
            RobotContainer.DBTelemetry.addData("Detected Color " + (i + 1), "null");
            RobotContainer.DBTelemetry.addData("Color " + (i + 1) + " Bounding Box",
                    "Top-left: (null, null), Bottom-right: (null, null)"
            );
        }
        // Update the telemetry to show the initial layout
        RobotContainer.DBTelemetry.update();
    }

    // Method to update the dashboard with actual detected values
    public void updateDashboard() {
        // Get the list of detected colors
        List<DetectedColorWithAngle> detectedColors = GetCurrentColAndAng();

        // Update each slot with the detected values or leave them as null if no detection
        for (int i = 0; i < 4; i++) { // Assuming a maximum of 10 color detections
            if (i < detectedColors.size()) {
                DetectedColorWithAngle detected = detectedColors.get(i);

                // Update detected color name and area in the telemetry
                RobotContainer.DBTelemetry.addData("Detected Color " + (i + 1), detected.colorName);

                // Update bounding box information
                RobotContainer.DBTelemetry.addData("Color " + (i + 1) + " Bounding Box",
                        "Top-left: (" + detected.boundingBox.x + ", " + detected.boundingBox.y + "), " +
                                "Bottom-right: (" + (detected.boundingBox.x + detected.boundingBox.width) + ", " +
                                (detected.boundingBox.y + detected.boundingBox.height) + ")"
                );
            } else {
                // No detection for this slot, set it to null or empty string
                RobotContainer.DBTelemetry.addData("Detected Color " + (i + 1), "null");
                RobotContainer.DBTelemetry.addData("Color " + (i + 1) + " Bounding Box",
                        "Top-left: (null, null), Bottom-right: (null, null)"
                );
            }
        }

        // Update the telemetry to reflect the latest data on the dashboard
        RobotContainer.DBTelemetry.update();
    }

    // get current AprilTag detections (if any) from camera
    // returns list containing info on each tag detected
    public List<DetectedColor> GetCurrentColorDetections() {
        return myColorDetectProcessor.getDetectedColors();
    }
    public List<DetectedColorWithAngle> GetCurrentColAndAng(){
        return myColorAndOrienDetProcessor.getDetectedColorsAndAng();
    }

    // get camera frames per second
    public double GetCameraFPS () {
        return CameraPortal.getFps();
    }

    // use to turn on/off AprilTag processing
    public void EnableColorDetectProcessing (boolean enable) {
        CameraPortal.setProcessorEnabled(myColorDetectProcessor, enable);
    }
}