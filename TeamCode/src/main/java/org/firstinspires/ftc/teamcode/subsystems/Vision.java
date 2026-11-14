package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AimConfig;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/*
 * VISION - Limelight3A: AprilTag targeting, auto-aim target, and the hive-tipped check.
 * Aim: the tags are grouped into 4 clusters (one per hive, see RobotConstants.TAG_*). The cluster
 * with the most tags in view wins (ties: closest to center). Its median tx/ty is smoothed over
 * the last few loops, and getAimTx()/getAimTy() return that. Ported from AimTestOdom.
 * Status/tx/ty/fiducial telemetry and the FTC Dashboard camera stream live here
 * (ported from the standalone LimelightTest bench OpMode).
 */
public class Vision implements Subsystem {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private Limelight3A limelight;
    private LLResult result;

    private Boolean tipped = false;
    private Servo servo;

    // Auto-aim
    private static final int SMOOTHING_WINDOW = 10;
    private static final String[] CLUSTER_NAMES =
            {"Red scoring", "Red audience", "Blue audience", "Blue scoring"};
    private final double[] txHistory = new double[SMOOTHING_WINDOW];
    private final double[] tyHistory = new double[SMOOTHING_WINDOW];
    private int historyIndex = 0;   // next slot to write
    private int historyCount = 0;
    private int selectedCluster = -1;
    private double aimTx, aimTy;    // smoothed

    public Vision(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    @Override
    public void init() {
        limelight = hardwareMap.get(
                Limelight3A.class,
                RobotConstants.LIMELIGHT
        );
        servo = hardwareMap.get(Servo.class, RobotConstants.LIME_LITE_SERVO);
        limelight.pipelineSwitch(0);
        limelight.start();
        FtcDashboard.getInstance().startCameraStream(limelight, 10);
    }

    @Override
    public void update() {
        result = limelight.getLatestResult();
        updateAim();

        LLStatus status = limelight.getStatus();
        telemetry.addData("Limelight", "temp %.1fC  cpu %.1f%%  fps %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        if (hasTarget()) {
            telemetry.addData("Limelight tx/ty", "%.2f / %.2f", getTx(), getTy());
        }
        if (hasAimTarget()) {
            telemetry.addData("Aim", "%s  tx %.2f  ty %.2f", getAimClusterName(), aimTx, aimTy);
        }
    }

    /** Tilts the Limelight. Only moves when called, so other OpModes leave the servo alone. */
    public void setServoPosition(double position) {
        servo.setPosition(position);
    }

    /** True if a tag cluster is in view. Only valid after update(). */
    public boolean hasAimTarget() {
        return selectedCluster != -1;
    }

    /** Smoothed horizontal angle to the chosen tag cluster, degrees. */
    public double getAimTx() {
        return aimTx;
    }

    /** Smoothed vertical angle to the chosen tag cluster, degrees. */
    public double getAimTy() {
        return aimTy;
    }

    public String getAimClusterName() {
        return hasAimTarget() ? CLUSTER_NAMES[selectedCluster] : "None";
    }

    private void updateAim() {
        List<LLResultTypes.FiducialResult> tags = new ArrayList<>();
        if (result != null && result.getFiducialResults() != null) {
            tags = result.getFiducialResults();
        }

        // Count the tags in each cluster and sum their tx.
        int[] count = new int[4];
        double[] txSum = new double[4];
        for (LLResultTypes.FiducialResult fr : tags) {
            int c = clusterOf(fr.getFiducialId());
            if (c != -1) {
                count[c]++;
                txSum[c] += fr.getTargetXDegrees();
            }
        }

        // Most tags wins. Ties go to the cluster closest to center.
        int best = -1;
        double bestOffCenter = Double.MAX_VALUE;
        for (int c = 0; c < 4; c++) {
            if (count[c] == 0) continue;
            double offCenter = Math.abs(txSum[c] / count[c]);
            if (best == -1 || count[c] > count[best]
                    || (count[c] == count[best] && offCenter < bestOffCenter)) {
                best = c;
                bestOffCenter = offCenter;
            }
        }

        if (best == -1) {
            // Lost the target: forget the history.
            selectedCluster = -1;
            historyIndex = 0;
            historyCount = 0;
            aimTx = 0;
            aimTy = 0;
            return;
        }

        if (best != selectedCluster) { // switched hives: old readings no longer apply
            historyIndex = 0;
            historyCount = 0;
        }
        selectedCluster = best;

        // Median of the chosen cluster's tags (ignores one bad tag).
        List<Double> txs = new ArrayList<>();
        List<Double> tys = new ArrayList<>();
        for (LLResultTypes.FiducialResult fr : tags) {
            if (clusterOf(fr.getFiducialId()) == best) {
                txs.add(fr.getTargetXDegrees());
                tys.add(fr.getTargetYDegrees());
            }
        }
        txHistory[historyIndex] = median(txs);
        tyHistory[historyIndex] = median(tys);
        historyIndex = (historyIndex + 1) % SMOOTHING_WINDOW;
        if (historyCount < SMOOTHING_WINDOW) historyCount++;

        // Weighted average, newest reading counts most.
        double txTotal = 0, tyTotal = 0, weightTotal = 0;
        int newest = (historyIndex - 1 + SMOOTHING_WINDOW) % SMOOTHING_WINDOW;
        for (int age = 0; age < historyCount; age++) {
            int i = (newest - age + SMOOTHING_WINDOW) % SMOOTHING_WINDOW;
            double weight = Math.pow(1.0 - AimConfig.SMOOTHING_ALPHA, age);
            txTotal += txHistory[i] * weight;
            tyTotal += tyHistory[i] * weight;
            weightTotal += weight;
        }
        aimTx = txTotal / weightTotal;
        aimTy = tyTotal / weightTotal;
    }

    /** 0 red scoring, 1 red audience, 2 blue audience, 3 blue scoring, -1 not a hive tag. */
    private static int clusterOf(int id) {
        if (id >= RobotConstants.TAG_RED_SCORING_MIN && id <= RobotConstants.TAG_RED_SCORING_MAX) return 0;
        if (id >= RobotConstants.TAG_RED_AUDIENCE_MIN && id <= RobotConstants.TAG_RED_AUDIENCE_MAX) return 1;
        if (id >= RobotConstants.TAG_BLUE_AUDIENCE_MIN && id <= RobotConstants.TAG_BLUE_AUDIENCE_MAX) return 2;
        if (id >= RobotConstants.TAG_BLUE_SCORING_MIN && id <= RobotConstants.TAG_BLUE_SCORING_MAX) return 3;
        return -1;
    }

    private static double median(List<Double> values) {
        Collections.sort(values);
        int n = values.size();
        return n % 2 == 0
                ? (values.get(n / 2 - 1) + values.get(n / 2)) / 2.0
                : values.get(n / 2);
    }

    public boolean hasTarget() {
        return result != null && result.isValid();
    }

    public double getTx() {
        return hasTarget() ? result.getTx() : 0;
    }

    public double getTy() {
        return hasTarget() ? result.getTy() : 0;
    }

    public void setPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    /**
     * True if a tag in [minId, maxId] is in view right now.
     * Use the RobotConstants.TAG_*_SCORING/AUDIENCE_MIN/MAX ranges (red and blue, both mapped).
     */
    public boolean isHiveTipped(int minId, int maxId) {
        tipped = false;

        if (hasTarget()) {
            for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                int id = fr.getFiducialId();
                if (id >= minId && id <= maxId) {
                    tipped = true;
                    break;
                }
            }
        }

        return tipped;
    }

    @Override
    public void stop() {
    }
}
