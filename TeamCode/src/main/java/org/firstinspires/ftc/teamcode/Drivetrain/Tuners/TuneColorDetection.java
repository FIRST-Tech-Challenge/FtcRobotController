package org.firstinspires.ftc.teamcode.Drivetrain.Tuners;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Test Color Detection", group = "Autonomous")
public class TuneColorDetection extends LinearOpMode {

    private Limelight3A limelight;
    private int currentPipeline = 2;  // Starting pipeline for detecting red

    @Override
    public void runOpMode() {
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        // Set the initial pipeline for red detection
        limelight.pipelineSwitch(currentPipeline);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            // Get color detection result
            String detectedColor = getColorResults();

            // Display the detected color
            telemetry.addData("Detected Color", detectedColor);

            // If no valid color is detected, cycle through the pipelines
            if (detectedColor.equals("None")) {
                telemetry.addData("Status", "No color detected, switching pipeline.");
                cyclePipeline();  // Switch to the next pipeline
            } else {
                // If a valid color is detected, stop switching
                telemetry.addData("Status", "Color detected.");
            }

            telemetry.update();
        }
    }

    /**
     * Cycles through the color detection pipelines: red -> blue -> yellow -> red...
     */
    public void cyclePipeline() {
        if (currentPipeline == 2) {
            setPipeline(3);  // Switch to blue detection
        } else if (currentPipeline == 3) {
            setPipeline(4);  // Switch to yellow detection
        } else if (currentPipeline == 4) {
            setPipeline(2);  // Switch back to red detection
        }
    }

    /**
     * Sets a new pipeline and updates the currentPipeline variable.
     *
     * @param pipeline The pipeline number to switch to.
     */
    public void setPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
        currentPipeline = pipeline;
    }

    /**
     * Returns the color detected by the Limelight based on the active pipeline.
     *
     * @return The color detected as a String ("Red", "Blue", "Yellow"), or "None" if no color is detected.
     */
    public String getColorResults() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Check the current pipeline to determine the detected color
            switch (currentPipeline) {
                case 2:
                    return "Red";   // Pipeline 2 detects red
                case 3:
                    return "Blue";  // Pipeline 3 detects blue
                case 4:
                    return "Yellow"; // Pipeline 4 detects yellow
                default:
                    return "Unknown";  // No specific color is associated with this pipeline
            }
        }
        return "None"; // No valid detection found
    }
}

