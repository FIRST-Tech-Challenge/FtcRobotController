package org.firstinspires.ftc.teamcode.PRL.Class;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class LimelightClass {

    private final Limelight3A limelight;
    private LLResult result;
    private boolean hasTarget = false;

    private double tx = 0;
    private double ty = 0;
    private double ta = 0;

    private double captureLatency = 0;
    private double targetingLatency = 0;

    private int tagID = -1;
    private Pose3D botPose = null;

    private int targetTagID = -1;

    public LimelightClass(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public void setPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    public void setTargetTagID(int tagID) {
        targetTagID = tagID;
    }

    public int getTargetTagID() {
        return targetTagID;
    }

    public void update() {
        result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            hasTarget = false;
            tagID = -1;
            return;
        }

        LLResultTypes.FiducialResult target = null;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (fiducials != null) {
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if (targetTagID < 0 || fiducial.getFiducialId() == targetTagID) {
                    target = fiducial;
                    break;
                }
            }
        }

        if (target == null) {
            hasTarget = false;
            tagID = -1;
            return;
        }

        hasTarget = true;
        tagID = target.getFiducialId();
        tx = target.getTargetXDegrees();
        ty = target.getTargetYDegrees();
        ta = target.getTargetArea();

        captureLatency = result.getCaptureLatency();
        targetingLatency = result.getTargetingLatency();

        botPose = result.getBotpose();
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    public double getTa() {
        return ta;
    }

    public int getTagID() {
        return tagID;
    }

    public Pose3D getBotPose() {
        return botPose;
    }

    public double getCaptureLatency() {
        return captureLatency;
    }

    public double getTargetingLatency() {
        return targetingLatency;
    }

    public LLResult getResult() {
        return result;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("LL Connected", limelight.isConnected());
        telemetry.addData("Has Target", hasTarget);
        telemetry.addData("Tag ID", tagID);

        telemetry.addData("tx", "%.2f", tx);
        telemetry.addData("ty", "%.2f", ty);
        telemetry.addData("ta", "%.2f", ta);

        telemetry.addData("Capture Latency", "%.2f", captureLatency);
        telemetry.addData("Target Latency", "%.2f", targetingLatency);

        if (botPose != null) {
            telemetry.addData("BotPose", botPose.toString());
        }
    }
}
