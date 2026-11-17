package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** Fresh tv and tx gate shared by the hardware autonomous shooting states. */
public final class ShootingAim {
    private ShootingAim() { }

    public static String turnSuggestion(boolean tvAuthorized, double tx) {
        if (!tvAuthorized) return "No turn: no fresh valid target (tv)";
        if (!Double.isFinite(tx)) return "No turn: invalid tx";
        if (Math.abs(tx) <= 2.0) return "Aligned: hold heading (within 2 degrees)";
        return String.format(java.util.Locale.US, "Turn %s %.1f degrees toward target",
                tx > 0 ? "CLOCKWISE / RIGHT" : "COUNTERCLOCKWISE / LEFT", Math.abs(tx));
    }

    public static boolean align(Sensors sensors, double now, Outputs outputs) {
        Vision vision = sensors.vision;
        outputs.shooterPower = outputs.feederPower = 0;
        outputs.turnClockwise = 0;
        if (!sensors.cameraConnected || !vision.fresh(now, 0)
                || now - vision.capturedAt > .25 || vision.targets.length == 0
                || !Double.isFinite(vision.tx)) {
            return false;
        }
        if (Math.abs(vision.tx) <= 2.0) {
            return true;
        }
        if (sensors.odometryValid && sensors.pose != null && sensors.pose.finite()
                && now - sensors.sampledAt >= 0 && now - sensors.sampledAt <= .25) {
            outputs.turnClockwise = FieldLayout.clip(vision.tx * .02, .25);
        }
        return false;
    }
}
