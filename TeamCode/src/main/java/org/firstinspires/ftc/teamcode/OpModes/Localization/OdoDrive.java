package org.firstinspires.ftc.teamcode.OpModes.Localization;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.follower.api.PoseFactory;
import org.firstinspires.ftc.teamcode.follower.drivetrain.DrivePowers;
import org.firstinspires.ftc.teamcode.follower.hardware.Mecanum;
import org.firstinspires.ftc.teamcode.follower.hardware.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Drive the robot around and watch the localizer. No follower involved, just
 * the gamepad, the motors and the Pinpoint, so any wrong number is the localizer.
 *
 * Quick check: put the robot on a tape mark, press A, drive anywhere, drive back
 * onto the mark. X, Y and Heading should all come back to about 0.
 *
 * Controls (gamepad 1):
 *   Left stick    = drive / strafe
 *   Right stick X = turn
 *   Right bumper  = half speed, for lining back up on the mark
 *   A             = zero the pose (0, 0, 0)
 *   B             = set the pose to FIELD_POSE, to check field coordinates
 */
@TeleOp(name = "Odo Drive", group = "Test")
public class OdoDrive extends LinearOpMode {

    // Same pose as the start of the auto, so B lets you compare against the field.
    private static final PoseFactory P = PoseFactory.degrees();
    private static final Pose FIELD_POSE = P.of(85, 132, 270);

    @Override
    public void runOpMode() throws InterruptedException {
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        Mecanum drive = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        // Same physical device the localizer uses, only read here for health info.
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class,
                Constants.localizerConfig.name.get());
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

        localizer.setPose(Pose.zero());

        telemetry.addLine("Odo Drive ready. Press A on your mark after start.");
        telemetry.update();

        waitForStart();

        localizer.setPose(Pose.zero());
        ElapsedTime loopTimer = new ElapsedTime();
        boolean lastA = false, lastB = false;

        while (opModeIsActive()) {
            localizer.update();

            // ---------- buttons (edge-detected so one press = one action) ----------
            if (gamepad1.a && !lastA) localizer.setPose(Pose.zero());
            if (gamepad1.b && !lastB) localizer.setPose(FIELD_POSE);
            lastA = gamepad1.a; lastB = gamepad1.b;

            // ---------- driving ----------
            double scale = gamepad1.right_bumper ? 0.5 : 1.0;
            drive.drive(new DrivePowers(
                    -gamepad1.left_stick_y * scale,
                    -gamepad1.left_stick_x * scale,
                    -gamepad1.right_stick_x * scale), true);

            // ---------- telemetry (Driver Station + Panels) ----------
            Pose pose = localizer.pose();
            line(panels, "X (in)", fmt(pose.x()));
            line(panels, "Y (in)", fmt(pose.y()));
            line(panels, "Heading (deg)", fmt(Math.toDegrees(pose.heading())));
            line(panels, "Forward speed (in/s)", fmt(localizer.twist().vx));
            line(panels, "Left speed (in/s)", fmt(localizer.twist().vy));
            line(panels, "Turn speed (deg/s)", fmt(Math.toDegrees(localizer.twist().omega)));
            line(panels, "Odo status", odo.getDeviceStatus());
            line(panels, "Odo rate (Hz)", fmt(odo.getFrequency()));
            line(panels, "Loop (ms)", fmt(loopTimer.milliseconds()));
            loopTimer.reset();

            telemetry.update();
            panels.update();
        }
        drive.stop();
    }

    private void line(TelemetryManager panels, String caption, Object value) {
        telemetry.addData(caption, value);
        panels.addData(caption, value);
    }

    private static String fmt(double v) {
        return String.format("%.2f", v);
    }
}
