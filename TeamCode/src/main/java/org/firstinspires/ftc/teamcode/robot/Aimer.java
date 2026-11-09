package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

/**
 * Aims the turret at our alliance's hive using the Limelight.
 *
 * The Limelight rides on the turret, so tx (the tag's left/right angle in the image) is exactly
 * how far the turret is off target. The turret encoder is only used for soft limits that keep
 * the turret from turning far enough to rip out its wires.
 *
 * Typical use, once per loop:
 * <pre>
 *     aimer.update();                 // read the latest Limelight frame
 *     if (wantToAim) aimer.aim();     // turn the turret toward the hive
 *     else aimer.stopTurret();
 * </pre>
 *
 * Setup: call {@link #resetTurretEncoder()} once, while the turret is lined up facing straight
 * backward (that position is 0), then {@link #start()} when the match starts.
 */
public class Aimer {

    // ---------------- Tuning / configuration ----------------
    public static final int PIPELINE = 0;

    // Hive tag IDs for each alliance. While a list is empty, ANY tag the Limelight sees is used.
    private static final int[] BLUE_HIVE_TAG_IDS = {
            38, 39, 40, 41, // Audience side
            42, 43, 44, 45  // Scoring side
    };
    private static final int[] RED_HIVE_TAG_IDS = {
            30, 31, 32, 33, // Scoring side
            34, 35, 36, 37  // Audience side
    };

    public static double TURRET_KP = 0.01;           // turret power per degree of tx
    public static double TURRET_MIN_POWER = 0.15;    // enough to overcome friction
    public static double TURRET_MAX_POWER = 0.8;
    public static double TURRET_AIM_DIRECTION = 1;   // flip to -1 if the turret turns away
    public static double AIM_TOLERANCE_DEG = 1.5;    // "on target" if |tx| is below this

    // Soft limits in encoder ticks, measured from the backward-facing start (0).
    public static int TURRET_MIN_TICKS = -428;
    public static int TURRET_MAX_TICKS = 428;

    public static double MAX_STALENESS_MS = 100;     // ignore results older than this
    private static final double METERS_TO_INCHES = 39.3701;

    // ---------------- Hardware and state ----------------
    private final DcMotor turret;
    private final Limelight3A limelight;
    private final Telemetry telemetry;

    private boolean blueAlliance = true;
    private HiveSighting hive = null;  // from the latest update(); null = no hive in view
    private boolean onTarget = false;
    private boolean atLimit = false;
    private String status = "idle";

    /** What we know about the hive from one Limelight frame. */
    private static class HiveSighting {
        double txDeg;          // average horizontal angle to the visible hive tags
        double distanceMeters; // average straight-line distance
        int tagCount;
    }

    public Aimer(DcMotor turret, Limelight3A limelight, Telemetry telemetry) {
        this.turret = turret;
        this.limelight = limelight;
        this.telemetry = telemetry;

        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // ---------------- Setup ----------------

    /** Make the turret's current position 0. Call while it faces straight backward. */
    public void resetTurretEncoder() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setBlueAlliance(boolean blue) {
        this.blueAlliance = blue;
    }

    public boolean isBlueAlliance() {
        return blueAlliance;
    }

    /** Start the Limelight. Call once, before the first update(). */
    public void start() {
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(PIPELINE);
    }

    /** Stop the turret and the Limelight. */
    public void stop() {
        stopTurret();
        limelight.stop();
    }

    // ---------------- Each loop ----------------

    /** Read the latest Limelight frame. Call once per loop, before aim() or any getter. */
    public void update() {
        hive = findHive(limelight.getLatestResult());
    }

    /**
     * Turn the turret toward the hive, but never past the soft limits.
     * Uses the frame from the last update().
     *
     * @return true if the turret is on target
     */
    public boolean aim() {
        if (hive == null) {
            // Can't see the hive: hold still rather than sweep around looking for it.
            turret.setPower(0);
            onTarget = false;
            atLimit = false;
            status = "NO TARGET - point the robot toward the hive";
            return false;
        }

        double error = hive.txDeg; // + means the target is to the right in the image
        onTarget = Math.abs(error) < AIM_TOLERANCE_DEG;

        double power = 0;
        if (!onTarget) {
            power = Range.clip(TURRET_KP * Math.abs(error), TURRET_MIN_POWER, TURRET_MAX_POWER);
            power *= TURRET_AIM_DIRECTION * Math.signum(error);
        }

        atLimit = wouldPassLimit(power);
        if (atLimit) power = 0;

        turret.setPower(power);

        if (atLimit) status = "AT LIMIT - turn the robot toward the hive";
        else status = onTarget ? "ON TARGET" : String.format("aiming %.2f", power);
        return onTarget;
    }

    /**
     * Drive the turret at a fixed power by hand, stopping at the soft limits. Use this for
     * driver control during a match.
     */
    public void moveTurret(double power) {
        onTarget = false;
        atLimit = wouldPassLimit(power);
        if (atLimit) power = 0;
        turret.setPower(power);
        status = atLimit ? "AT LIMIT" : String.format("manual %.2f", power);
    }

    /**
     * Soft limits: true if this power would push the turret further past a limit. Turning back
     * toward center is always allowed. (Positive power counts the encoder up; setDirection
     * reverses both together.)
     */
    private boolean wouldPassLimit(double power) {
        int ticks = getTurretTicks();
        return (power > 0 && ticks >= TURRET_MAX_TICKS)
                || (power < 0 && ticks <= TURRET_MIN_TICKS);
    }

    /** Stop the turret (it brakes and holds where it is). */
    public void stopTurret() {
        turret.setPower(0);
        onTarget = false;
        atLimit = false;
        status = "idle";
    }

    /**
     * Drive the turret at a fixed power by hand. This IGNORES the soft limits, on purpose, so it
     * can be used to calibrate them -- watch the wires. For match driving use moveTurret().
     */
    public void nudgeTurret(double power) {
        turret.setPower(power);
        onTarget = false;
        atLimit = false;
        status = String.format("MANUAL %.2f", power);
    }

    // ---------------- What the aimer knows ----------------

    /** True if the last update() saw at least one of our hive tags. */
    public boolean hasTarget() {
        return hive != null;
    }

    /** True if the last aim() ended on target. */
    public boolean isOnTarget() {
        return onTarget;
    }

    /** Average tx of the visible hive tags in degrees (+ = right), or 0 with no target. */
    public double getTxDegrees() {
        return hive == null ? 0 : hive.txDeg;
    }

    /** Average straight-line distance to the visible hive tags, or 0 with no target. */
    public double getDistanceInches() {
        return hive == null ? 0 : hive.distanceMeters * METERS_TO_INCHES;
    }

    public int getTagCount() {
        return hive == null ? 0 : hive.tagCount;
    }

    public int getTurretTicks() {
        return turret.getCurrentPosition();
    }

    public boolean isPastLimit() {
        int ticks = getTurretTicks();
        return ticks < TURRET_MIN_TICKS || ticks > TURRET_MAX_TICKS;
    }

    /** A short description of what the turret is doing, for telemetry. */
    public String getStatus() {
        return status;
    }

    /** Add the aimer's state to telemetry. The caller still calls telemetry.update(). */
    public void showStatus() {
        telemetry.addData("Turret", status);
        telemetry.addData("Turret ticks", "%d   (soft limits %d to %d)%s",
                getTurretTicks(), TURRET_MIN_TICKS, TURRET_MAX_TICKS,
                isPastLimit() ? "  PAST LIMIT" : "");
        telemetry.addData("Alliance", blueAlliance ? "BLUE" : "RED");
        if (hive != null) {
            telemetry.addData("Hive tags seen", hive.tagCount);
            telemetry.addData("tx (deg)", "%.2f", hive.txDeg);
            // Straight-line distance is still right even though the camera turns with the
            // turret; the separate x, y, z are relative to wherever the turret points.
            telemetry.addData("Distance (in)", "%.1f", getDistanceInches());
        } else {
            telemetry.addData("Hive", "No hive tags seen");
        }
    }

    // ---------------- Finding the hive ----------------

    /**
     * Look through every tag in the frame, keep the ones on our alliance's hive,
     * and average them so we aim at the middle of the group of tags we can see.
     * Returns null if no usable tags.
     */
    private HiveSighting findHive(LLResult result) {
        if (result == null || !result.isValid() || result.getStaleness() > MAX_STALENESS_MS) {
            return null;
        }
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null) return null;

        HiveSighting sighting = new HiveSighting();
        for (LLResultTypes.FiducialResult tag : tags) {
            if (!isOurHiveTag(tag.getFiducialId())) continue;

            Position p = tag.getTargetPoseRobotSpace().getPosition(); // meters
            sighting.txDeg += tag.getTargetXDegrees();
            sighting.distanceMeters += Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            sighting.tagCount++;
        }
        if (sighting.tagCount == 0) return null;

        sighting.txDeg /= sighting.tagCount;
        sighting.distanceMeters /= sighting.tagCount;
        return sighting;
    }

    private boolean isOurHiveTag(int id) {
        int[] ids = blueAlliance ? BLUE_HIVE_TAG_IDS : RED_HIVE_TAG_IDS;
        if (ids.length == 0) return true; // IDs not filled in yet: accept any tag
        for (int good : ids) {
            if (good == id) return true;
        }
        return false;
    }
}
