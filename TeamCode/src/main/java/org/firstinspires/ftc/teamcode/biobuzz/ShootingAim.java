package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** Hive image-center calculation and fresh tx gate for autonomous shooting states. */
public final class ShootingAim {
    public static final double ALIGNMENT_TOLERANCE_DEGREES=2.0;
    public static final double TURN_GAIN_PER_DEGREE=.02;
    public static final double MAX_TURN_POWER=.25;
    public static final int MIN_VISIBLE_TAGS=2;
    public static final int REQUIRED_STABLE_FRAMES=3;
    public static final double REQUIRED_STABLE_SECONDS=.10;
    private ShootingAim() { }

    /** A negative alliance selects the single visible family for robot-relative routes. */
    public static Vision centerOnHive(Vision vision, int[][] tagGroups, int alliance) {
        double center = Double.NaN;
        int aimTagCount = 0;
        if (vision.valid && vision.pipeline == 0) {
            for (int family = 0; family < tagGroups.length; family++) {
                if (alliance >= 0 && family != alliance) continue;
                double sum = 0;
                int count = 0;
                for (int id : tagGroups[family]) {
                    Target seen = null;
                    for (Target target : vision.targets) {
                        if (target.id != id) continue;
                        if (seen != null || !Double.isFinite(target.tx) || Math.abs(target.tx) >= 90) {
                            return new Vision(vision.valid, vision.pipeline, vision.capturedAt, vision.targets, Double.NaN, 0);
                        }
                        seen = target;
                    }
                    if (seen != null) {
                        // Average image-plane positions, not angular offsets or apparent tag sizes.
                        sum += Math.tan(Math.toRadians(seen.tx));
                        count++;
                    }
                }
                if (count == 0) continue;
                if (Double.isFinite(center)) {
                    return new Vision(vision.valid, vision.pipeline, vision.capturedAt, vision.targets, Double.NaN, 0);
                }
                center = Math.toDegrees(Math.atan(sum / count));
                aimTagCount = count;
            }
        }
        return new Vision(vision.valid, vision.pipeline, vision.capturedAt, vision.targets, center, aimTagCount);
    }

    public static String turnSuggestion(boolean tvAuthorized, double tx) {
        if (!tvAuthorized) return "No turn: no fresh valid target (tv)";
        if (!Double.isFinite(tx)) return "No turn: invalid tx";
        if (Math.abs(tx) <= ALIGNMENT_TOLERANCE_DEGREES + 1e-9) return "Aligned: hold heading (within 2 degrees)";
        return String.format(java.util.Locale.US, "Turn %s %.1f degrees toward target",
                tx > 0 ? "CLOCKWISE / RIGHT" : "COUNTERCLOCKWISE / LEFT", Math.abs(tx));
    }

    public static final class Alignment {
        private int stableFrames;
        private double stableSince=Double.NaN,lastFrame=Double.NaN;

        public void reset() {
            stableFrames=0;stableSince=lastFrame=Double.NaN;
        }

        public boolean align(Sensors sensors,double now,Outputs outputs) {
            Vision vision=sensors.vision;
            outputs.shooterPower=outputs.feederPower=0;
            outputs.forward=outputs.strafeRight=0;
            outputs.turnClockwise=0;
            if(!sensors.cameraConnected||!vision.fresh(now,0)||now-vision.capturedAt>.25
                    ||vision.targets.length==0||vision.aimTagCount<MIN_VISIBLE_TAGS||!Double.isFinite(vision.tx)){
                reset();return false;
            }
            if(Math.abs(vision.tx)>ALIGNMENT_TOLERANCE_DEGREES+1e-9){
                reset();
                if(sensors.odometryValid&&sensors.pose!=null&&sensors.pose.finite()
                        &&now-sensors.sampledAt>=0&&now-sensors.sampledAt<=.25)
                    outputs.turnClockwise=FieldLayout.clip(vision.tx*TURN_GAIN_PER_DEGREE,MAX_TURN_POWER);
                return false;
            }
            if(vision.capturedAt==lastFrame)return stableFrames>=REQUIRED_STABLE_FRAMES
                    &&vision.capturedAt-stableSince+1e-9>=REQUIRED_STABLE_SECONDS;
            if(Double.isFinite(lastFrame)&&vision.capturedAt<lastFrame){reset();return false;}
            if(Double.isFinite(lastFrame)&&vision.capturedAt-lastFrame>.25)reset();
            if(stableFrames==0)stableSince=vision.capturedAt;
            stableFrames++;lastFrame=vision.capturedAt;
            return stableFrames>=REQUIRED_STABLE_FRAMES
                    &&vision.capturedAt-stableSince+1e-9>=REQUIRED_STABLE_SECONDS;
        }
    }
}
