package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;

public class TipDetects { //need to write as NextFTC subsystem

    public enum HiveState {
        SCORING_CELL_UP,
        AUDIENCE_CELL_UP,
        UNKNOWN
    }

    public enum Alliance {
        RED,
        BLUE
    }

    private Limelight3A limelight;
    private HiveState currentState = HiveState.UNKNOWN;
    private Alliance alliance;
    private ElapsedTime watchdogTimer;

    private int currentScoringTags = 0;
    private int currentAudienceTags = 0;

    private static final double WATCHDOG_TIMEOUT_SEC = 10.0;
    //private static final double CAMERA_HEIGHT_METERS = 15.0 * 0.0254; // 15 inches
    //private static final double CAMERA_PITCH_DEG = 35.0;

    public TipDetects(HardwareMap hardwareMap, String deviceName, Alliance alliance) {
        this.limelight = hardwareMap.get(Limelight3A.class, deviceName);
        this.alliance = alliance;
        this.watchdogTimer = new ElapsedTime();

        limelight.pipelineSwitch(0);
        limelight.start();

        watchdogTimer.reset();
    }

    public void update() {
        LLResult result = limelight.getLatestResult();

        currentScoringTags = 0;
        currentAudienceTags = 0;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

            for (LLResultTypes.FiducialResult tag : tags) {
                int id = tag.getFiducialId();
                if (isScoringTag(id)) {
                    currentScoringTags++;
                } else if (isAudienceTag(id)) {
                    currentAudienceTags++;
                }
            }

            if (currentScoringTags >= 3) {
                currentState = HiveState.SCORING_CELL_UP;
                watchdogTimer.reset();
            } else if (currentAudienceTags >= 3) {
                currentState = HiveState.AUDIENCE_CELL_UP;
                watchdogTimer.reset();
            }
        }

        if (watchdogTimer.seconds() > WATCHDOG_TIMEOUT_SEC) {
            currentState = HiveState.UNKNOWN;
        }
    }

    private boolean isScoringTag(int id) {
        if (alliance == Alliance.RED) return id >= 30 && id <= 33;
        if (alliance == Alliance.BLUE) return id >= 42 && id <= 45;
        return false;
    }

    private boolean isAudienceTag(int id) {
        if (alliance == Alliance.RED) return id >= 34 && id <= 37;
        if (alliance == Alliance.BLUE) return id >= 38 && id <= 41;
        return false;
    }

    public HiveState getState() {
        return currentState;
    }

    public int getDetectedClusterTags() {
        return currentScoringTags + currentAudienceTags;
    }

    public double getWatchdogTime() {
        return watchdogTimer.seconds();
    }

    public void stop() {
        limelight.stop();
    }
}