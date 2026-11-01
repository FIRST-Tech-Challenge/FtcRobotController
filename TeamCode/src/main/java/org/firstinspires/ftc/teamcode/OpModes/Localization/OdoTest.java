package org.firstinspires.ftc.teamcode.OpModes.Localization;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.teamcode.follower.api.PoseFactory;
import org.firstinspires.ftc.teamcode.follower.follower.Follower;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Odometry check: shows Pedro's follower pose next to the raw Pinpoint values.
 *
 * Controls (gamepad 1):
 *   Left stick   = drive / strafe  (only when DRIVE is enabled)
 *   Right stick X= turn
 *   Y            = toggle driving on/off (starts OFF so you can push by hand)
 *   A            = reset pose to START_POSE
 *   B            = reset pose to (0, 0, 0)
 */
@TeleOp(name = "Odo Test", group = "Test")
public class OdoTest extends LinearOpMode {

    // Set this to where you place the robot, same as the auto's start pose.
    private static final PoseFactory P = PoseFactory.degrees();
    private static final Pose START_POSE = P.of(85, 132, 270);

    // If the follower pose and raw odo differ by more than this, flag it.
    private static final double MATCH_TOL_IN = 0.1;
    private static final double MATCH_TOL_DEG = 0.5;

    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);
        // Same physical device the follower uses. The follower already calls
        // pinpoint.update() inside follower.update(), so we only read here.
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class,
                Constants.localizerConfig.name.get());
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

        follower.setPose(START_POSE);
        Pose reference = START_POSE;   // deltas are measured from here

        boolean driveEnabled = false;
        boolean lastY = false, lastA = false, lastB = false;

        telemetry.addLine("Odo Test ready. Robot starts in PUSH mode (no driving).");
        telemetry.addLine("Press Y after start to enable gamepad driving.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            // ---------- buttons (edge-detected so one press = one action) ----------
            if (gamepad1.y && !lastY) driveEnabled = !driveEnabled;
            if (gamepad1.a && !lastA) { follower.setPose(START_POSE); reference = START_POSE; }
            if (gamepad1.b && !lastB) { Pose zero = P.of(0, 0, 0); follower.setPose(zero); reference = zero; }
            lastY = gamepad1.y; lastA = gamepad1.a; lastB = gamepad1.b;

            // ---------- driving ----------
            if (driveEnabled) {
                follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            } else {
                follower.manual(0, 0, 0);
            }

            // ---------- read both sources ----------
            Pose f = follower.pose();
            double fX = f.x(), fY = f.y(), fH = Math.toDegrees(f.heading());

            double oX = odo.getPosX(DistanceUnit.INCH);
            double oY = odo.getPosY(DistanceUnit.INCH);
            double oH = odo.getHeading(AngleUnit.DEGREES);

            double headDiff = wrapDeg(fH - oH);
            boolean match = Math.abs(fX - oX) <= MATCH_TOL_IN
                    && Math.abs(fY - oY) <= MATCH_TOL_IN
                    && Math.abs(headDiff) <= MATCH_TOL_DEG;

            // Movement since last reset, in field coords and robot coords.
            double dX = fX - reference.x();
            double dY = fY - reference.y();
            double dH = wrapDeg(fH - Math.toDegrees(reference.heading()));
            // Rotate the field delta into the robot's starting frame:
            // +forward = where the robot was facing at reset, +left = its left.
            double refH = reference.heading();
            double dForward =  dX * Math.cos(refH) + dY * Math.sin(refH);
            double dLeft    = -dX * Math.sin(refH) + dY * Math.cos(refH);

            // ---------- telemetry (Driver Station + Panels) ----------
            line(panels, "Mode", driveEnabled ? "DRIVE (Y to stop)" : "PUSH BY HAND (Y to drive)");
            line(panels, "Odo status", odo.getDeviceStatus());

            line(panels, "--- Follower (Pedro) ---", "");
            line(panels, "F X (in)", fmt(fX));
            line(panels, "F Y (in)", fmt(fY));
            line(panels, "F Heading (deg)", fmt(fH));

            line(panels, "--- Raw Pinpoint ---", "");
            line(panels, "O X (in)", fmt(oX));
            line(panels, "O Y (in)", fmt(oY));
            line(panels, "O Heading (deg)", fmt(oH));
            line(panels, "Enc X (ticks)", odo.getEncoderX());
            line(panels, "Enc Y (ticks)", odo.getEncoderY());

            line(panels, "Follower = Odo?", match ? "YES" : "NO  <-- check localizer setup");

            line(panels, "--- Since last reset ---", "");
            line(panels, "Field dX / dY (in)", fmt(dX) + " / " + fmt(dY));
            line(panels, "Robot forward (in)", fmt(dForward) + "   push forward -> should go UP");
            line(panels, "Robot left (in)", fmt(dLeft) + "   push left -> should go UP");
            line(panels, "Turn (deg)", fmt(dH) + "   turn CCW -> should go UP");

            line(panels, "Follower busy", follower.isBusy());
            line(panels, "Loop (Hz)", fmt(odo.getFrequency()));

            telemetry.update();
            panels.update();
        }
    }

    private void line(TelemetryManager panels, String caption, Object value) {
        telemetry.addData(caption, value);
        panels.addData(caption, value);
    }

    private static String fmt(double v) {
        return String.format("%.2f", v);
    }

    /** Wrap degrees to -180..180. */
    private static double wrapDeg(double d) {
        while (d > 180) d -= 360;
        while (d <= -180) d += 360;
        return d;
    }
}


