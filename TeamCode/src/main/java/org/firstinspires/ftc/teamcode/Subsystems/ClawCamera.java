package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Size;
import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.vision.ColorAndOrientationDetect;
import org.firstinspires.ftc.teamcode.vision.DetectedColorWithAngle;
import org.firstinspires.ftc.vision.VisionPortal;
import java.util.List;

/** Subsystem */
public class ClawCamera extends SubsystemBase {

    private ColorAndOrientationDetect myColorAndOrienDetProcessor;
    // Local objects and variables here
    private final VisionPortal CameraPortal;

    private boolean dashboardInitialized = false;

    /** Place code here to initialize subsystem */
    public ClawCamera(String cameraName) {
        myColorAndOrienDetProcessor = new ColorAndOrientationDetect();
        myColorAndOrienDetProcessor.setMinBoundingBoxArea(0.05);
        CameraPortal = new VisionPortal.Builder()
                .setCamera(RobotContainer.ActiveOpMode.hardwareMap.get(WebcamName.class, cameraName))
                .addProcessors(myColorAndOrienDetProcessor)
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
        for (int i = 0; i < 4; i++) { // Assuming a maximum of 4 color detections
            RobotContainer.DBTelemetry.addData("Detected Color " + (i + 1), "null");
            RobotContainer.DBTelemetry.addData("Color " + (i + 1) + " Center", "(null, null)");
            RobotContainer.DBTelemetry.addData("Color " + (i + 1) + " Angle", "null");
            RobotContainer.DBTelemetry.addData("Color " + (i + 1) + " YBR", "[Y=null, B=null, R=null]");
        }
        // Update the telemetry to show the initial layout
        RobotContainer.DBTelemetry.update();
    }

    // Method to update the dashboard with actual detected values
    private void updateDashboard() {
        // Assume detectedColors is a list of DetectedColorWithAngle objects containing the detected values
        List<DetectedColorWithAngle> detectedColors = GetCurrentColAndAng();
        for (int i = 0; i < 4; i++) {
            if (i < detectedColors.size()) {
                DetectedColorWithAngle detectedColor = detectedColors.get(i);
                // Update the dashboard with the actual detected values
                RobotContainer.DBTelemetry.addData("Detected Color " + (i + 1), detectedColor.getColor());
                RobotContainer.DBTelemetry.addData("Color " + (i + 1) + " Center",
                        "(" + detectedColor.getCenter().x + ", " + detectedColor.getCenter().y + ")");
                RobotContainer.DBTelemetry.addData("Color " + (i + 1) + " Angle", detectedColor.getAngle());
                RobotContainer.DBTelemetry.addData("Color " + (i + 1) + " YBR",
                        "[Y=" + detectedColor.getColor()[0] + ", B=" + detectedColor.getColor()[1] + ", R=" + detectedColor.getColor()[2] + "]");
            } else {
                // If there are fewer detections than slots, fill remaining slots with "null"
                RobotContainer.DBTelemetry.addData("Detected Color " + (i + 1), "null");
                RobotContainer.DBTelemetry.addData("Color " + (i + 1) + " Center", "(null, null)");
                RobotContainer.DBTelemetry.addData("Color " + (i + 1) + " Angle", "null");
                RobotContainer.DBTelemetry.addData("Color " + (i + 1) + " YBR", "[Y=null, B=null, R=null]");
            }
        }
        // Update the telemetry to reflect new data
        RobotContainer.DBTelemetry.update();
    }

    // returns list containing info on each specimen detected
    public List<DetectedColorWithAngle> GetCurrentColAndAng(){
        return myColorAndOrienDetProcessor.getDetectedColorsAndAng();
    }

    // Returns true if the specified color is detected in the frame
    public boolean isColorExist(String colorName) {
        return myColorAndOrienDetProcessor.isColorExist(colorName);
    }

    // Returns the angle of the most centered bounding box for the specified color
    // Returns Double.NaN if the color is not detected
    public double getAngle(String colorName) {
        return myColorAndOrienDetProcessor.getAngle(colorName);
    }

    // Returns the center (x, y) of the most centered bounding box for the specified color
    // Returns {Double.NaN, Double.NaN} if the color is not detected
    public double[] getCenter(String colorName) {
        return myColorAndOrienDetProcessor.getCenter(colorName);
    }

    // use to turn on/off color and angle detect processing
    public void EnableDetectProcessing (boolean enable) {
        CameraPortal.setProcessorEnabled(myColorAndOrienDetProcessor, enable);
    }

}