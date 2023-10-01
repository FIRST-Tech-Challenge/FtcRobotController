package org.firstinspires.ftc.teampractice.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teampractice.AutonomousBase;

@Autonomous(name = "HSV Test", group = "Example", preselectTeleOp="Gamepad")
public class HSVTest extends AutonomousBase {
    HSVPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        pipeline = new HSVPipeline();
        pipeline.setupWebcam(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Analysis", () -> String.format("%3.0f", pipeline.data));

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            telemetry.update();
            idle();
        }

        pipeline.webcam.stopStreaming();
    }
}
