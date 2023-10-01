package org.firstinspires.ftc.teampractice.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teampractice.AutonomousBase;

@Autonomous(name = "Yellow Test", group = "Example", preselectTeleOp="Gamepad")
public class YellowTest extends AutonomousBase {
    YellowPipeline pipeline;
    @Override
    public void runOpMode() throws InterruptedException {
        pipeline = new YellowPipeline();
        pipeline.setupWebcam(hardwareMap);

        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            telemetry.update();
            idle();
        }

        pipeline.webcam.stopStreaming();
    }
}
