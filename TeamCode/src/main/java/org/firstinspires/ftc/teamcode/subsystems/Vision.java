package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

/*
 * VISION - Limelight3A: AprilTag targeting and the hive-tipped check.
 * Status/tx/ty/fiducial telemetry and the FTC Dashboard camera stream live here
 * (ported from the standalone LimelightTest bench OpMode).
 */
public class Vision implements Subsystem {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private Limelight3A limelight;
    private LLResult result;

    private Boolean tipped = false;

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

        limelight.pipelineSwitch(0);
        limelight.start();
        FtcDashboard.getInstance().startCameraStream(limelight, 10);
    }

    @Override
    public void update() {
        result = limelight.getLatestResult();

        LLStatus status = limelight.getStatus();
        telemetry.addData("Limelight", "temp %.1fC  cpu %.1f%%  fps %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        if (hasTarget()) {
            telemetry.addData("Limelight tx/ty", "%.2f / %.2f", getTx(), getTy());
        }
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
