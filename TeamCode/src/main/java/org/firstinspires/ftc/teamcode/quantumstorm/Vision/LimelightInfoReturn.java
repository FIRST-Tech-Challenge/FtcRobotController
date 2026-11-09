package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

import java.util.List;

@Autonomous(name = "Limelight: AprilTag Auto Monitor", group = "Vision")
public class LimelightInfoReturn extends LinearOpMode {

    private static final double MAX_TAG_AGE_MS = 250;
    private static final double PIPELINE_SETTLE_SECONDS = 0.3;

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.tryGet(Limelight3A.class, Constants.LIMELIGHT);
        if (limelight == null) {
            telemetry.addLine("Limelight not found in Robot Configuration.");
            telemetry.addData("tv", 0);
            telemetry.update();
            return;
        }

        IMU imu = hardwareMap.tryGet(IMU.class, "imu");
        if (imu != null) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        }

        ElapsedTime pipelineTimer = new ElapsedTime();
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(Constants.LIMELIGHT_APRILTAG_PIPELINE);
        limelight.start();
        pipelineTimer.reset();
        telemetry.addLine("AprilTag pipeline selected. Press PLAY to monitor targets; no drive motion.");
        telemetry.update();

        try {
            waitForStart();
            while (opModeIsActive()) {
                double imuYaw = Double.NaN;
                if (imu != null) {
                    YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
                    imuYaw = angles.getYaw(AngleUnit.DEGREES);
                    limelight.updateRobotOrientation(imuYaw);
                }
                LLResult result = limelight.getLatestResult();
                LLResultTypes.FiducialResult target = null;
                int visibleTagCount = 0;

                if (limelight.isConnected() && pipelineTimer.seconds() >= PIPELINE_SETTLE_SECONDS
                        && result != null && result.isValid()
                        && result.getPipelineIndex() == Constants.LIMELIGHT_APRILTAG_PIPELINE
                        && result.getStaleness() >= 0 && result.getStaleness() <= MAX_TAG_AGE_MS) {
                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                    if (tags != null) {
                        visibleTagCount = tags.size();
                        for (LLResultTypes.FiducialResult tag : tags) {
                            if (target == null || Math.abs(tag.getTargetXDegrees())
                                    < Math.abs(target.getTargetXDegrees())) {
                                target = tag;
                            }
                        }
                    }
                }

                int tv = target == null ? 0 : 1;
                double tx = target == null ? Double.NaN : target.getTargetXDegrees();
                double ty = target == null ? Double.NaN : target.getTargetYDegrees();
                String action = "Monitoring only; no drive commands";

                telemetry.addData("Limelight", limelight.isConnected() ? "Connected" : "Disconnected");
                telemetry.addData("Pipeline", Constants.LIMELIGHT_APRILTAG_PIPELINE + " (AprilTag)");
                telemetry.addData("tv", tv);
                telemetry.addData("AprilTags detected", visibleTagCount);
                telemetry.addData("Target ID", target == null ? "None" : target.getFiducialId());
                telemetry.addData("tx (degrees)", Double.isFinite(tx) ? String.format("%.2f", tx) : "N/A");
                telemetry.addData("ty (degrees)", Double.isFinite(ty) ? String.format("%.2f", ty) : "N/A");
                telemetry.addData("Drive action", action);
                telemetry.addData("IMU yaw", Double.isFinite(imuYaw) ? String.format("%.1f°", imuYaw) : "Not configured");
                telemetry.addData("Image + processing latency (ms)", result == null ? "N/A"
                        : String.format("%.1f", result.getCaptureLatency() + result.getTargetingLatency()));
                addMegaTagTelemetry(result);
                telemetry.update();
                sleep(20);
            }
        } finally {
            limelight.stop();
        }
    }

    private void addMegaTagTelemetry(LLResult result) {
        telemetry.addLine("MegaTag2 needs a loaded field map and configured camera-to-robot pose.");
        if (result == null || !result.isValid() || result.getBotposeTagCount() < 1) {
            telemetry.addData("MegaTag2 botpose", "Unavailable");
            return;
        }

        Pose3D botpose = result.getBotpose_MT2();
        if (botpose == null || botpose.getPosition() == null || botpose.getOrientation() == null) {
            telemetry.addData("MegaTag2 botpose", "Unavailable");
            return;
        }

        Position inches = botpose.getPosition().toUnit(DistanceUnit.INCH);
        telemetry.addData("MegaTag2 X / Y / Z (in)", "%.1f / %.1f / %.1f",
                inches.x, inches.y, inches.z);
        telemetry.addData("MegaTag2 yaw (degrees)", "%.1f",
                botpose.getOrientation().getYaw(AngleUnit.DEGREES));
        telemetry.addData("MegaTag2 tags used", result.getBotposeTagCount());
    }
}
