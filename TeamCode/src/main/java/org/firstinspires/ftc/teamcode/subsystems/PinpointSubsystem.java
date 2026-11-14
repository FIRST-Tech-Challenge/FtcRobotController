package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/*
 * Pinpoint Subsystem
 * Owns the goBILDA PINPOINT odometry computer and its two odometry pods. It only READS. It never
 * moves a motor, and it has no gamepad code. Another class (for now TeleOp - Alpha 14 and
 * PinpointAlpha) asks it where the robot is.
 *
 * WHAT IT ANSWERS: "Where is the robot, and which way is it facing, compared to where it started?"
 *   getXInches()          -> how far FORWARD of the start point. Forward is POSITIVE.
 *   getYInches()          -> how far LEFT of the start point. Left is POSITIVE.
 *   getHeadingDegrees()   -> which way the robot faces. Turning LEFT (counterclockwise) is POSITIVE.
 * Those are the same signs as DriveSubsystem and TurretSubsystem (forward +, left +, counter-
 * clockwise +), and the same as Pedro Pathing. So the turret's field direction is just:
 *   shooterFieldHeading = robotHeading + turretAngle
 * (HiveBearingMemory uses this, so the turret can keep pointing at the hive when the camera
 * loses it.)
 *
 * HOW IT WORKS: two spring-loaded (swingarm) odometry pods ride on the floor. One pod faces
 * forward (it counts when the robot drives forward or backward) and one faces sideways (it counts
 * when the robot strafes). The Pinpoint adds in its own gyro (the IMU) to work out the turning.
 * The pods are not wheels that can slip, so this is more accurate than counting drive wheel turns.
 * It still drifts a little over a whole match, so treat it as "close", not "exact".
 *
 * STATE vs STATUS: this class gives the robot no commands, so there is no state. The STATUS is
 * what the Pinpoint reports about itself (getDeviceStatus()): READY means it is working.
 *
 * ABSENT DEVICE: if the robot's configuration has no device named PINPOINT_NAME, this class does
 * NOT stop the OpMode. Every getter answers 0, isAvailable() is false, and the telemetry says
 * what to fix. (The robot can still drive. It just does not know where it is.)
 *
 * DROP LOG (a black box for finding why the Pinpoint stops saying READY): every time the status
 * goes from READY to something else, update() records the story of that "drop": which statuses it
 * went through, how long it lasted, and how far the HEADING moved while it was happening. The
 * screen shows it, and every drop is also written to the Robot Controller log (see BiobuzzLog), so
 * it can be read after the match. It changes nothing the robot does. HOW TO READ IT:
 *   - FAULT_X_POD / FAULT_Y_POD / FAULT_NO_PODS: a pod cable is loose or a pod is failing. The gyro
 *     (and so the heading) may carry on fine, so look at "heading moved": a small number means the
 *     heading survived.
 *   - FAULT_BAD_READ: the I2C cable or connector (a bad read between the Hub and the Pinpoint).
 *   - NOT_READY then CALIBRATING: the Pinpoint RESTARTED (a power or reset problem). Its heading is
 *     back at a new zero, so "heading moved" will be large.
 *   - FAULT_IMU_RUNAWAY: the gyro went wrong.
 * The log starts only after the Pinpoint has said READY once, so the normal start-up is not a drop.
 *
 * The OpMode must call update() ONCE per loop(). Nothing calls it for us.
 *
 * Robot configuration (on the Driver Station):
 *   I2C Device name: pinpoint   (type "goBILDA Pinpoint Odometry Computer", plugged into one of the
 *                                Control Hub's I2C ports 1, 2 or 3. Port 0 is the Hub's own IMU.)
 *   Last season this was named "odo". It has to be renamed once on each robot. The full steps are
 *   in TeamCode/docs/pinpoint-setup-guide.md.
 *
 * NOTE FOR AUTO: in Autonomous, Pedro Pathing reads the Pinpoint by itself (its localizer). Never
 * use this class and a Pedro Follower in the same OpMode, because both would talk to the device.
 */
public class PinpointSubsystem {

    // ------------------------------------------------------------------
    // Hardware name. This MUST match the robot configuration exactly.
    //
    // The name, the two pod offsets, the pod type and the two pod directions below are PUBLIC on
    // purpose. In Autonomous, pedro/Constants.java reads them to set up Pedro's own Pinpoint
    // localizer, so TeleOp and Auto always use the SAME numbers and each number is typed in only
    // ONE place: here. Change them here and both TeleOp and Auto get the change.
    // ------------------------------------------------------------------
    public static final String PINPOINT_NAME = "pinpoint";

    // ------------------------------------------------------------------
    // Where the pods are. These numbers are copied from last season's code (same chassis), and
    // were measured then. TODO (issue #48): Measure them again with a tape measure if either pod has
    // been moved. Wrong numbers do NOT hurt the heading, but the X and Y readings drift sideways
    // whenever the robot turns.
    // ------------------------------------------------------------------
    // Measured from the robot's center of turning, to the middle of each pod's wheel.
    //
    // The FORWARD pod (it counts forward and backward) sits to one side of the center.
    //   LEFT of center is POSITIVE, RIGHT of center is NEGATIVE.
    //   Ours is 3.75 inches to the RIGHT, so it is -3.75.
    public static final double FORWARD_POD_SIDEWAYS_OFFSET_INCHES = -3.75;

    // The SIDEWAYS pod (it counts left and right) sits ahead of or behind the center.
    //   FORWARD of center is POSITIVE, BEHIND center is NEGATIVE.
    //   Ours is centered front to back, so it is 0.0.
    public static final double SIDEWAYS_POD_FORWARD_OFFSET_INCHES = 0.0;

    // ------------------------------------------------------------------
    // The pods themselves
    // ------------------------------------------------------------------
    // goBILDA makes two kinds of odometry pod, and each counts a different number of ticks per
    // millimeter. Ours are the spring-loaded SWINGARM pods. (The other kind is goBILDA_4_BAR_POD.)
    // If the readings are off by a steady amount, check this first.
    public static final GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;

    // Which way each pod counts. The rule is: X must go UP when the robot is pushed FORWARD, and
    // Y must go UP when the robot is pushed LEFT. If either one goes the wrong way, change that
    // one to REVERSED. (Last season both were FORWARD.)
    // TESTED on the robot: the forward pod was right as FORWARD. The sideways pod counted
    // NEGATIVE when the robot was pushed LEFT, so it is REVERSED. (The pod is mounted with its
    // thick side toward the front, which is the opposite of last season. It does not need to be
    // flipped by hand, because this setting does the same job.)
    public static final GoBildaPinpointDriver.EncoderDirection FORWARD_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection SIDEWAYS_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    // ------------------------------------------------------------------
    // Corrections found by testing on the robot
    // ------------------------------------------------------------------
    // Heading must go UP when the robot turns LEFT (counterclockwise). The Pinpoint can be
    // mounted right side up or upside down, and that can flip the sign. If turning the robot LEFT
    // makes the heading go DOWN, change this to true.
    // TESTED on the robot: turning RIGHT gave a negative heading and turning LEFT a positive one,
    // so false is right. (Last season's helper flipped it, but the Pedro Pathing setup did not.)
    private static final boolean INVERT_HEADING = false;

    // Distance fix. Push the robot a measured distance (say 48 inches) and see what X says.
    //   multiplier = real distance / distance the screen shows
    // Start at 1.0. Last season's code used 1.5 (it showed 6 inches for a real 9). We did NOT copy
    // that, because 1.5 is exactly the ratio between the two pod settings (19.894 / 13.263), so it
    // probably covered up a pod setting that was not taking effect. Measure first, and if it is
    // still off by a steady amount, put the number here. It is applied to X, Y and their speeds.
    private static final double DISTANCE_CALIBRATION_MULTIPLIER = 1.0;

    // ------------------------------------------------------------------
    // The hardware
    // ------------------------------------------------------------------
    // null when the robot's configuration has no device with PINPOINT_NAME.
    private final GoBildaPinpointDriver pinpoint;

    // The newest answers, saved by update(). They are zero when there is no Pinpoint.
    private double xInches = 0.0;
    private double yInches = 0.0;
    private double headingDegrees = 0.0;
    private double xSpeedInchesPerSecond = 0.0;
    private double ySpeedInchesPerSecond = 0.0;
    private double headingSpeedDegreesPerSecond = 0.0;

    // ------------------------------------------------------------------
    // The drop log (see DROP LOG at the top). Only describes what happened. Nothing reads it to
    // make a decision.
    // ------------------------------------------------------------------
    // Longest story we keep for one drop. A drop that wanders through many statuses is cut off.
    private static final int DROP_STORY_MAX_CHARACTERS = 90;

    private boolean hasBeenReady = false;          // the log starts the first time it says READY
    private boolean inDrop = false;                // true from the loop it stops saying READY
    private GoBildaPinpointDriver.DeviceStatus lastStatusInDrop = null;
    private final ElapsedTime dropTimer = new ElapsedTime();
    // The heading on the last loop the status was READY. A drop is measured against THIS, not the
    // first not-READY reading: a restart can zero the heading at the very moment it stops saying
    // READY, and that first reading would already be the new zero.
    private double lastReadyHeadingDegrees = 0.0;
    private double dropStartHeadingDegrees = 0.0;
    private int dropCount = 0;
    private String lastDropStory = "";             // for example "FAULT_X_POD_NOT_DETECTED > READY"
    private double lastDropSeconds = 0.0;          // while a drop is running: the seconds so far
    private double lastDropHeadingChangeDegrees = 0.0;
    private double longestDropSeconds = 0.0;

    // ==================================================================
    // SETUP
    // ==================================================================

    /*
     * Gets the Pinpoint, tells it where the pods are, and zeros the position and the gyro.
     * The robot must be STILL when this runs (at INIT), because the gyro calibrates now.
     */
    public PinpointSubsystem(HardwareMap hardwareMap) {
        // tryGet gives back null instead of stopping the OpMode when the name is not in the
        // robot configuration.
        pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, PINPOINT_NAME);
        if (pinpoint == null) {
            return;
        }

        pinpoint.setOffsets(FORWARD_POD_SIDEWAYS_OFFSET_INCHES,
                SIDEWAYS_POD_FORWARD_OFFSET_INCHES, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(POD_TYPE);
        pinpoint.setEncoderDirections(FORWARD_POD_DIRECTION, SIDEWAYS_POD_DIRECTION);

        // Zero the position and the heading, and recalibrate the gyro.
        pinpoint.resetPosAndIMU();
    }

    // ==================================================================
    // ACTIONS: the OpMode tells the Pinpoint what to do
    // ==================================================================

    /*
     * Call this ONCE per loop(). It reads the Pinpoint and saves the newest answers.
     */
    public void update() {
        if (pinpoint == null) {
            return;
        }
        pinpoint.update();

        xInches = pinpoint.getPosX(DistanceUnit.INCH) * DISTANCE_CALIBRATION_MULTIPLIER;
        yInches = pinpoint.getPosY(DistanceUnit.INCH) * DISTANCE_CALIBRATION_MULTIPLIER;
        headingDegrees = pinpoint.getHeading(AngleUnit.DEGREES);
        xSpeedInchesPerSecond = pinpoint.getVelX(DistanceUnit.INCH) * DISTANCE_CALIBRATION_MULTIPLIER;
        ySpeedInchesPerSecond = pinpoint.getVelY(DistanceUnit.INCH) * DISTANCE_CALIBRATION_MULTIPLIER;
        headingSpeedDegreesPerSecond = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

        if (INVERT_HEADING) {
            headingDegrees = -headingDegrees;
            headingSpeedDegreesPerSecond = -headingSpeedDegreesPerSecond;
        }

        updateDropLog();
    }

    /*
     * Writes the drop log (see DROP LOG at the top). Called by update(), after the newest heading
     * has been saved. It only watches the status and writes down what it sees.
     */
    private void updateDropLog() {
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();

        if (status == GoBildaPinpointDriver.DeviceStatus.READY) {
            if (inDrop) {
                // It is back. Close the story.
                inDrop = false;
                lastDropSeconds = dropTimer.seconds();
                lastDropHeadingChangeDegrees = HiveBearingMemory.wrapDegrees(headingDegrees - dropStartHeadingDegrees);
                addToDropStory(status);
                if (lastDropSeconds > longestDropSeconds) {
                    longestDropSeconds = lastDropSeconds;
                }
                // Also write it to the Robot Controller log, so it can be read after the match.
                BiobuzzLog.info("Pinpoint drop #%d ended: %.2f s, heading moved %.0f deg, story: %s",
                        dropCount, lastDropSeconds, lastDropHeadingChangeDegrees, lastDropStory);
            }
            hasBeenReady = true;
            lastReadyHeadingDegrees = headingDegrees;
            return;
        }

        if (!hasBeenReady) {
            // Still starting up for the first time. That is not a drop.
            return;
        }

        if (!inDrop) {
            // A drop just started.
            inDrop = true;
            dropCount = dropCount + 1;
            dropTimer.reset();
            dropStartHeadingDegrees = lastReadyHeadingDegrees;
            lastDropStory = status.toString();
            lastStatusInDrop = status;
            BiobuzzLog.warning("Pinpoint drop #%d STARTED: status %s (last READY heading %.1f deg)",
                    dropCount, status.toString(), lastReadyHeadingDegrees);
        } else if (status != lastStatusInDrop) {
            // Still not READY, but the status changed (for example a fault, then CALIBRATING).
            addToDropStory(status);
            lastStatusInDrop = status;
        }

        // Keep the numbers current while the drop is still going on.
        lastDropSeconds = dropTimer.seconds();
        lastDropHeadingChangeDegrees = HiveBearingMemory.wrapDegrees(headingDegrees - dropStartHeadingDegrees);
    }

    /*
     * Adds " > STATUS" to the story of the current drop, unless the story is already long.
     */
    private void addToDropStory(GoBildaPinpointDriver.DeviceStatus status) {
        if (lastDropStory.length() < DROP_STORY_MAX_CHARACTERS) {
            lastDropStory = lastDropStory + " > " + status.toString();
        }
    }

    /*
     * Makes the robot's CURRENT spot the new start point: X = 0, Y = 0, heading = 0. This is
     * quick and safe any time, even while driving. It does NOT recalibrate the gyro.
     */
    public void resetPose() {
        if (pinpoint != null) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0.0, 0.0, AngleUnit.DEGREES, 0.0));
        }
        xInches = 0.0;
        yInches = 0.0;
        headingDegrees = 0.0;
        xSpeedInchesPerSecond = 0.0;
        ySpeedInchesPerSecond = 0.0;
        headingSpeedDegreesPerSecond = 0.0;
    }

    /*
     * Zeros the position AND recalibrates the gyro. The robot must be STILL (the gyro needs about
     * a quarter of a second to settle). Use it if the heading has drifted while the robot sat.
     */
    public void recalibrate() {
        if (pinpoint != null) {
            pinpoint.resetPosAndIMU();
        }
        xInches = 0.0;
        yInches = 0.0;
        headingDegrees = 0.0;
        xSpeedInchesPerSecond = 0.0;
        ySpeedInchesPerSecond = 0.0;
        headingSpeedDegreesPerSecond = 0.0;
    }

    // ==================================================================
    // QUESTIONS: the OpMode asks where the robot is
    // ==================================================================

    /*
     * Inches FORWARD of the start point. Forward is POSITIVE.
     */
    public double getXInches() {
        return xInches;
    }

    /*
     * Inches LEFT of the start point. Left is POSITIVE.
     */
    public double getYInches() {
        return yInches;
    }

    /*
     * Which way the robot faces, in degrees, from -180 to 180. Turning LEFT (counterclockwise) is
     * POSITIVE. Zero is the way the robot faced at the start.
     */
    public double getHeadingDegrees() {
        return headingDegrees;
    }

    /*
     * How fast the robot is going forward (positive) or backward (negative), in inches per second.
     */
    public double getXSpeedInchesPerSecond() {
        return xSpeedInchesPerSecond;
    }

    /*
     * How fast the robot is going left (positive) or right (negative), in inches per second.
     */
    public double getYSpeedInchesPerSecond() {
        return ySpeedInchesPerSecond;
    }

    /*
     * How fast the robot is turning, in degrees per second. Turning LEFT is POSITIVE.
     */
    public double getHeadingSpeedDegreesPerSecond() {
        return headingSpeedDegreesPerSecond;
    }

    /*
     * The raw tick counts of the two pods. They ignore every correction in this class (the
     * offsets, the multiplier, the heading flip) AND the pod direction setting. Tested on the
     * robot: a sideways pod set to REVERSED, with Y correct (+ going left), shows its ticks going
     * DOWN. So use them only to see that each pod is counting at all. Judge the DIRECTIONS by
     * getXInches() and getYInches(). Zero when there is no Pinpoint.
     */
    public int getForwardPodTicks() {
        if (pinpoint == null) {
            return 0;
        }
        return pinpoint.getEncoderX();
    }

    public int getSidewaysPodTicks() {
        if (pinpoint == null) {
            return 0;
        }
        return pinpoint.getEncoderY();
    }

    /*
     * True when the robot's configuration has a device with PINPOINT_NAME.
     */
    public boolean isAvailable() {
        return pinpoint != null;
    }

    /*
     * The Pinpoint's own report on itself: READY, CALIBRATING, NOT_READY, or one of the FAULT_
     * values. When the device is missing we say NOT_READY.
     */
    public GoBildaPinpointDriver.DeviceStatus getDeviceStatus() {
        if (pinpoint == null) {
            return GoBildaPinpointDriver.DeviceStatus.NOT_READY;
        }
        return pinpoint.getDeviceStatus();
    }

    /*
     * True only when the Pinpoint is there AND says READY. Check this before trusting the pose.
     */
    public boolean isReady() {
        return pinpoint != null && pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    /*
     * How many times per second the Pinpoint itself updates. Zero when there is no Pinpoint.
     */
    public double getPinpointFrequencyHz() {
        if (pinpoint == null) {
            return 0.0;
        }
        return pinpoint.getFrequency();
    }

    /*
     * Shows the Pinpoint's health and where the robot is. Call it after update(). It prints its
     * own heading, so the OpMode has nothing else to call.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- ODOMETRY (Pinpoint) ---");

        if (pinpoint == null) {
            telemetry.addData("Pinpoint", "NOT FOUND. Add an I2C device named \"%s\" to the robot config", PINPOINT_NAME);
            return;
        }

        telemetry.addData("Pinpoint status", getStatusText());
        telemetry.addData("Pose", "X %.1f in (+ forward)   Y %.1f in (+ left)   Heading %.1f deg (+ left)",
                xInches, yInches, headingDegrees);
        telemetry.addData("Speed", "X %.1f in/s   Y %.1f in/s   Turn %.1f deg/s",
                xSpeedInchesPerSecond, ySpeedInchesPerSecond, headingSpeedDegreesPerSecond);
        telemetry.addData("Pinpoint update rate", "%.0f Hz", getPinpointFrequencyHz());

        // The drop log. Write down what it says after a drop (a photo of the screen is fine).
        if (dropCount == 0) {
            telemetry.addData("Pinpoint drops", "none since it first said READY");
        } else {
            String nowText = "";
            if (inDrop) {
                nowText = "   *** DROPPED RIGHT NOW ***";
            }
            telemetry.addData("Pinpoint drops", "%d in total   longest %.2f s%s", dropCount, longestDropSeconds, nowText);
            telemetry.addData("Last drop", "%.2f s   heading moved %.0f deg   %s",
                    lastDropSeconds, lastDropHeadingChangeDegrees, lastDropStory);
        }
    }

    // ==================================================================
    // PRIVATE HELPERS
    // ==================================================================

    /*
     * The status as a sentence a driver can act on.
     */
    private String getStatusText() {
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        if (status == GoBildaPinpointDriver.DeviceStatus.READY) {
            return "READY";
        }
        if (status == GoBildaPinpointDriver.DeviceStatus.CALIBRATING) {
            return "CALIBRATING (hold the robot still)";
        }
        if (status == GoBildaPinpointDriver.DeviceStatus.NOT_READY) {
            return "NOT READY (it may still be starting up)";
        }
        if (status == GoBildaPinpointDriver.DeviceStatus.FAULT_NO_PODS_DETECTED) {
            return "FAULT: no pods detected (check both pod cables)";
        }
        if (status == GoBildaPinpointDriver.DeviceStatus.FAULT_X_POD_NOT_DETECTED) {
            return "FAULT: forward (X) pod not detected (check its cable)";
        }
        if (status == GoBildaPinpointDriver.DeviceStatus.FAULT_Y_POD_NOT_DETECTED) {
            return "FAULT: sideways (Y) pod not detected (check its cable)";
        }
        if (status == GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ) {
            return "FAULT: bad read (check the I2C cable)";
        }
        if (status == GoBildaPinpointDriver.DeviceStatus.FAULT_IMU_RUNAWAY) {
            return "FAULT: gyro runaway (recalibrate with the robot still)";
        }
        return status.toString();
    }
}
