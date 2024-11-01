package org.firstinspires.ftc.teamcode.Drivetrain.Tuners;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Test Color Detection", group = "Autonomous")
public class TuneColorDetection extends LinearOpMode {

    private Limelight3A limelight;
    private int currentPipeline = 1;  // Variable to keep track of the active pipeline

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        // Initialize the pipeline to 1 or any desired starting pipeline
        limelight.pipelineSwitch(currentPipeline);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            // Get the detected color using the new method
            String detectedColor = getColorResults();

            // Display the detected color
            telemetry.addData("Detected Color", detectedColor);

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    // Get position and size of detected color
                    double tx = result.getTx();  // X offset
                    double ty = result.getTy();  // Y offset
                    double ta = result.getTa();  // Area

                    // Display other color detection data on telemetry
                    telemetry.addData("tx", tx);
                    telemetry.addData("ty", ty);
                    telemetry.addData("Area", ta);
                }
            }
            telemetry.update();
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
                    return "Red";   // Pipeline 2 is set for detecting red
                case 3:
                    return "Blue";  // Pipeline 3 is set for detecting blue
                case 4:
                    return "Yellow"; // Pipeline 4 is set for detecting yellow
                default:
                    return "Unknown";  // No specific color is associated with this pipeline
            }
        }
        return "None"; // No valid detection found
    }
}



