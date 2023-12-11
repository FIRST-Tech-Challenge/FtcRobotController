package org.firstinspires.ftc.team417_CENTERSTAGE.apriltags;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseOpMode;

/*
    This test class is for measuring the latency for April Tag detection.
    It lights a green LED when it detects an April tag.
*/

@Disabled
@TeleOp (name = "LatencyTest")
public class AprilTagLatencyTest extends BaseOpMode {
    // Declares instance of our April Tag Pose Estimator
    AprilTagPoseEstimator myAprilTagLatencyCompensation;

    @Override
    public void runOpMode() {
        initializeHardware();

        // Turn red light to be off at beginning
        //     DigitalChannel object for LEDs makes this counterintuitive, on = false, off = true
        red.setState(true);

        myAprilTagLatencyCompensation = new AprilTagPoseEstimator(hardwareMap, telemetry);

        myAprilTagLatencyCompensation.init();

        waitForStart();

        while (opModeIsActive()) {
            // Code added to draw the pose
            TelemetryPacket p = new TelemetryPacket();
            Canvas c = p.fieldOverlay();

            myAprilTagLatencyCompensation.updatePoseEstimate();

            FtcDashboard dashboard = FtcDashboard.getInstance();
            dashboard.sendTelemetryPacket(p);
        }
    }
}
