package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.PinpointOdometry;

/**
 * Pedro's Pinpoint procedure, ported from Pedro's revhub localizer to V1's PinpointOdometry
 * device lookup and 4-Bar pod resolution. It drives no motors: push or spin the robot by hand.
 * Results are PinpointOdometry constants: setOffsets(x, y) in mm, the same convention as
 * Pedro's PinpointConfig xPodOffset/yPodOffset.
 */
public class PinpointTuner extends Procedure {
    /** Offsets assume a half turn; reject spins far from 180 degrees. */
    static final double MIN_OFFSET_SPIN_DEG = 150;

    public PinpointTuner() {
        super("Pinpoint Tuner", "A procedure for checking the Pinpoint pod directions and offsets.");
    }

    private Pose2D measure(PinpointPush step) throws InterruptedException {
        return runOpMode(step);
    }

    @Override
    public void run() throws InterruptedException {
        Pose2D forward = measure(new PinpointPush("Forward Direction Identification",
                "Determines if your forward pod needs to be reversed. \nPush your robot forward and then stop the Opmode",
                EncoderDirection.FORWARD, EncoderDirection.FORWARD));
        boolean forwardPodReversed = forward.getX(DistanceUnit.INCH) < 0;

        Pose2D left = measure(new PinpointPush("Strafe Direction Identification",
                "Determines if your strafe pod needs to be reversed. \nPush your robot to the left and then stop the Opmode",
                EncoderDirection.FORWARD, EncoderDirection.FORWARD));
        boolean strafePodReversed = left.getY(DistanceUnit.INCH) < 0;

        EncoderDirection xDirection = forwardPodReversed ? EncoderDirection.REVERSED : EncoderDirection.FORWARD;
        EncoderDirection yDirection = strafePodReversed ? EncoderDirection.REVERSED : EncoderDirection.FORWARD;

        PinpointPush spinStep = new PinpointPush("Offsets Identification",
                "Automatically identifies the offsets for your Pinpoint localizer. \nKeep the robot still until the Driver Station shows \"Measuring\", then spin it in place 180 degrees counterclockwise and stop the Opmode",
                xDirection, yDirection);
        Pose2D spun = measure(spinStep);
        double spinDeg = spun.getHeading(AngleUnit.DEGREES);
        if (Math.abs(spinDeg) < MIN_OFFSET_SPIN_DEG)
            abort(String.format("The robot turned %.1f degrees; spin it about 180 degrees counterclockwise in place and try again. [%s]",
                    spinDeg, spinStep.diagnostics));
        // A half turn about the centre moves each zero-offset pod by twice its offset.
        double xOffsetMm = -spun.getY(DistanceUnit.MM) / 2.0;
        double yOffsetMm = -spun.getX(DistanceUnit.MM) / 2.0;

        result("X_POD_DIRECTION", xDirection);
        result("Y_POD_DIRECTION", yDirection);
        result("PINPOINT_X_POD_OFFSET_MM", xOffsetMm);
        result("PINPOINT_Y_POD_OFFSET_MM", yOffsetMm);
        result("Offset spin (deg)", spinDeg);
        result("X_POD_DIRECTION (installed)", PinpointOdometry.X_POD_DIRECTION);
        result("Y_POD_DIRECTION (installed)", PinpointOdometry.Y_POD_DIRECTION);
        result("PINPOINT_X_POD_OFFSET_MM (installed)", PinpointOdometry.PINPOINT_X_POD_OFFSET_MM);
        result("PINPOINT_Y_POD_OFFSET_MM (installed)", PinpointOdometry.PINPOINT_Y_POD_OFFSET_MM);

        code(Language.JAVA, "// PinpointOdometry.java (compare with the measured pod geometry before replacing it)\n" +
                "public static final double PINPOINT_X_POD_OFFSET_MM = " + xOffsetMm + ";\n" +
                "public static final double PINPOINT_Y_POD_OFFSET_MM = " + yOffsetMm + ";\n" +
                "public static final GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION =\n" +
                "        GoBildaPinpointDriver.EncoderDirection." + xDirection + ";\n" +
                "public static final GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION =\n" +
                "        GoBildaPinpointDriver.EncoderDirection." + yDirection + ";");
    }
}

/** Zero pod offsets and the given directions; reports the last pose read before STOP. */
class PinpointPush extends TuningOpMode<Pose2D> {
    private static final Pose2D ZERO = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
    private final EncoderDirection xDirection, yDirection;
    /** What the Pinpoint reported during the measuring loop, for the abort message. */
    String diagnostics = "measuring loop never ran (STOP pressed before the IMU settled?)";

    PinpointPush(String name, String description, EncoderDirection xDirection, EncoderDirection yDirection) {
        super(name, description, true);
        this.xDirection = xDirection;
        this.yDirection = yDirection;
    }

    /** Status must read READY this long after the single IMU reset before the pose is zeroed. */
    static final double SETTLE_SECONDS = .5;
    static final double SETTLE_TIMEOUT_SECONDS = 5;

    @Override
    protected Pose2D runTuningOpMode() throws InterruptedException {
        // PinpointOdometry supplies V1's device name/aliases and 4-Bar resolution; override the rest.
        // Its constructor already resets the IMU; wait that out before overriding, then reset exactly once
        // more. A second reset sent while the first calibration runs can leave the heading untracked.
        GoBildaPinpointDriver pinpoint = new PinpointOdometry(hardwareMap).getDriver();
        waitForReady(pinpoint);
        pinpoint.setOffsets(0, 0, DistanceUnit.MM);
        pinpoint.setEncoderDirections(xDirection, yDirection);
        pinpoint.resetPosAndIMU();
        // Pedro's web tuner starts this OpMode as soon as it is confirmed, so hold the robot still
        // here until the IMU has settled; only then is it zeroed and the push/spin measured.
        waitForReady(pinpoint);
        waitForStart();
        pinpoint.setPosition(ZERO);
        pinpoint.update();

        // Report X/Y and the heading accumulated from the unnormalized heading (a spin past +/-180 degrees
        // still counts) from the last READY read only. STOP interrupts this thread; an I2C read cut off by it
        // returns all zeros, which on current firmware passes the SDK's corruption check and reads as
        // NOT_READY with pose 0, 0, 0. Keeping that final read made a real 180 degree spin report 0.
        Pose2D pose = ZERO;
        double startHeadingRad = pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS);
        int startX = pinpoint.getEncoderX(), startY = pinpoint.getEncoderY();
        int firmware = pinpoint.getDeviceVersion();
        double spunRad = 0, maxHeadingVelDeg = 0, maxNormHeadingDeg = 0;
        int loops = 0, rejected = 0;
        java.util.Set<String> statuses = new java.util.LinkedHashSet<>();
        while (!isStopRequested()) {
            pinpoint.update();
            if (isStopRequested()) break; // This read may be the interrupted, zeroed one.
            loops++;
            GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
            statuses.add(String.valueOf(status));
            if (status != GoBildaPinpointDriver.DeviceStatus.READY) {
                rejected++;
                continue;
            }
            Pose2D read = pinpoint.getPosition();
            spunRad = pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS) - startHeadingRad;
            maxHeadingVelDeg = Math.max(maxHeadingVelDeg,
                    Math.abs(pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)));
            maxNormHeadingDeg = Math.max(maxNormHeadingDeg, Math.abs(read.getHeading(AngleUnit.DEGREES)));
            diagnostics = String.format(java.util.Locale.US,
                    "loops %d, non-READY reads ignored %d, statuses %s, start heading %.1f deg, max |heading| %.1f deg, "
                            + "max |heading vel| %.1f deg/s, pod ticks moved X %d Y %d, firmware v%d",
                    loops, rejected, statuses, Math.toDegrees(startHeadingRad), maxNormHeadingDeg, maxHeadingVelDeg,
                    pinpoint.getEncoderX() - startX, pinpoint.getEncoderY() - startY, firmware);
            pose = new Pose2D(DistanceUnit.MM, read.getX(DistanceUnit.MM), read.getY(DistanceUnit.MM),
                    AngleUnit.RADIANS, spunRad);
            telemetry.addLine("Measuring: push or spin the robot now, then STOP.");
            telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
            telemetry.addData("Pose (in, deg)", "%.2f, %.2f | %.1f", pose.getX(DistanceUnit.INCH),
                    pose.getY(DistanceUnit.INCH), Math.toDegrees(spunRad));
            telemetry.addData("Heading velocity (deg/s)", "%.1f",
                    pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            telemetry.addData("Raw pods (ticks)", "X %d | Y %d", pinpoint.getEncoderX(), pinpoint.getEncoderY());
            telemetry.update();
        }
        return pose;
    }

    /** Polls until the Pinpoint has read READY for SETTLE_SECONDS; keep the robot still meanwhile. */
    private void waitForReady(GoBildaPinpointDriver pinpoint) throws InterruptedException {
        ElapsedTime total = new ElapsedTime(), ready = new ElapsedTime();
        boolean wasReady = false;
        Thread.sleep(50); // Let the reset command take effect before the first status read.
        while (!isStopRequested() && total.seconds() < SETTLE_TIMEOUT_SECONDS) {
            pinpoint.update();
            GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
            boolean isReady = status == GoBildaPinpointDriver.DeviceStatus.READY;
            if (isReady && !wasReady) ready.reset();
            wasReady = isReady;
            if (isReady && ready.seconds() >= SETTLE_SECONDS) return;
            telemetry.addLine("KEEP THE ROBOT STILL: calibrating the Pinpoint IMU");
            telemetry.addData("Pinpoint status", status);
            telemetry.update();
            Thread.sleep(20);
        }
        if (!isStopRequested())
            throw new IllegalStateException("Pinpoint did not report READY within " + SETTLE_TIMEOUT_SECONDS
                    + " s (status " + pinpoint.getDeviceStatus() + "); check its cable and configuration.");
    }
}
