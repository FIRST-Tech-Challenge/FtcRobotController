package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.utils.Constants;

import dev.nextftc.robot.Mechanism;

public final class LimelightController implements Mechanism {
    private static Limelight3A limelight;
    private static double lastAimTxDeg = Double.NaN;
    private static long lastTargetSeenNanos;
    private static int pipelineNumber = -1;

    public void initialize(HardwareMap hardwareMap, int initialPipeline) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        pipelineNumber = -1;
        selectPipeline(initialPipeline);
    }

    public void start() {
        limelight.start();
    }

    public void selectPipeline(int pipeline) {
        if (pipeline == pipelineNumber) {
            return;
        }
        pipelineNumber = pipeline;
        lastAimTxDeg = Double.NaN;
        limelight.pipelineSwitch(pipelineNumber);
    }

    public void stop() {
        lastAimTxDeg = Double.NaN;
        if (limelight != null) {
            limelight.stop();
            limelight = null;
        }
    }

    public static double getTx(){
        LLResult result = limelight.getLatestResult();
        boolean valid = result != null && result.isValid();
        if (!valid){
            return 0;
        }
        return result.getTx();
    }

    public static boolean seesArpilTag(){
        LLResult result = limelight.getLatestResult();
        boolean valid = result != null && result.isValid();
        return valid;
    }

    /** Returns the last valid tx for a short dropout, or NaN when it expires. */
    public static double getAimTx() {
        if (limelight == null) {
            lastAimTxDeg = Double.NaN;
            return Double.NaN;
        }

        LLResult result = limelight.getLatestResult();
        long now = System.nanoTime();
        if (result != null && result.isValid()
                && result.getPipelineIndex() == pipelineNumber
                && Double.isFinite(result.getTx())) {
            long ageMillis = Math.max(0L, result.getStaleness());
            if (ageMillis / 1000.0 < Constants.LIMELIGHT_AIM_HOLD_SECONDS) {
                lastAimTxDeg = result.getTx();
                // Re-reading an old frame must not restart the hold interval.
                lastTargetSeenNanos = now - ageMillis * 1_000_000L;
            }
        }

        double secondsSinceLastTarget = (now - lastTargetSeenNanos) / 1e9;
        if (Double.isFinite(lastAimTxDeg)
                && secondsSinceLastTarget < Constants.LIMELIGHT_AIM_HOLD_SECONDS) {
            return lastAimTxDeg;
        }
        return Double.NaN;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Limelight pipeline", pipelineNumber);
        LLResult result = limelight.getLatestResult();
        boolean valid = result != null && result.isValid();
        telemetry.addData("Vision target valid", valid);
        if (!valid) {
            return;
        }
        telemetry.addData("tx", "%.2f", result.getTx());
        telemetry.addData("ty", "%.2f", result.getTy());
        Pose3D botpose = result.getBotpose();
        if (botpose != null) {
            telemetry.addData("Limelight botpose", botpose);
        }
    }
}
