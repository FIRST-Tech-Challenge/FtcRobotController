package org.firstinspires.ftc.teamcode.quantumstorm;

import org.firstinspires.ftc.teamcode.biobuzz.*;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

public class StateMachineTest {
    public static void main(String[] args) {
        Outputs outputs = new Outputs();
        check(ShootingAim.align(sample(true, 0, 10, 2), 10, outputs), "2-degree boundary permits shooting");
        check(!ShootingAim.align(sample(true, 0, 10, 2.01), 10, outputs)
                && outputs.turnClockwise > 0, "positive tx turns clockwise");
        check(!ShootingAim.align(sample(true, 0, 10, -30), 10, outputs)
                && outputs.turnClockwise == -.25, "negative tx turns counterclockwise with bounded power");
        check(!ShootingAim.align(sample(false, 0, 10, 0), 10, outputs)
                && outputs.turnClockwise == 0, "tv=false never turns or shoots");
        check(!ShootingAim.align(sample(true, 1, 10, 0), 10, outputs), "color pipeline cannot authorize shooting");
        check(ShootingAim.align(sample(true, 0, 9.75, 0), 10, outputs), "250ms freshness boundary");
        check(!ShootingAim.align(sample(true, 0, 9.749, 0), 10, outputs), "stale frame blocks shooting");
        check(!ShootingAim.align(sample(true, 0, 10, Double.NaN), 10, outputs), "nonfinite tx blocks shooting");
        check(ShootingAim.turnSuggestion(false, 30).startsWith("No turn"), "tv denies directional suggestions");
        check(ShootingAim.turnSuggestion(true, 12).contains("CLOCKWISE / RIGHT"), "positive tx suggests right");
        check(ShootingAim.turnSuggestion(true, -12).contains("COUNTERCLOCKWISE / LEFT"), "negative tx suggests left");
        check(ShootingAim.turnSuggestion(true, 2).startsWith("Aligned"), "aligned target suggests holding heading");
        check(ShootingAim.turnSuggestion(true, Double.NaN).startsWith("No turn"), "nonfinite tx has no suggestion");
        testAutonomous(false);
        testAutonomous(true);
        System.out.println("Shooting aim and both autonomous gates passed");
    }

    private static Sensors sample(boolean tv, int pipeline, double capturedAt, double tx) {
        Target[] targets = {new Target(34, 0, 0)};
        return new Sensors(new Pose(0, 0, 0), 10, true, true, -1,
                new Vision(tv, pipeline, capturedAt, targets, tx));
    }

    private static void testAutonomous(boolean teamTwo) {
        FieldLayout field = new FieldLayout();
        FakeIO io = new FakeIO();
        io.pose = teamTwo ? field.teamTwoWallHive[0] : field.hive[0];
        io.tagId = field.nearTags[0][0];
        RobotController controller;
        if (teamTwo) {
            TeamTwoHiveAuto auto = new TeamTwoHiveAuto(io, field);
            auto.aimForShooting = true;
            auto.spinUpSeconds = 0;
            controller = auto;
        } else {
            TeamOnePollenAuto auto = new TeamOnePollenAuto(io, field);
            auto.aimForShooting = true;
            auto.spinUpSeconds = 0;
            controller = auto;
        }
        controller.start(0, .54);
        io.tx = 12;
        controller.tick();
        check(io.outputs.feederPower == 0 && io.outputs.turnClockwise > 0, "autonomous aligns before feed");
        io.tx = 0;
        for (int i = 0; i < 3; i++) { io.now += .05; controller.tick(); }
        check(io.outputs.feederPower > 0, "aligned live target permits feed");
        io.tx = -12;
        io.now += .05;
        controller.tick();
        check(io.outputs.feederPower == 0 && io.outputs.shooterPower == 0
                && io.outputs.turnClockwise < 0, "alignment loss pauses feeding and corrects heading");
        io.tv = false;
        io.now += .05;
        controller.tick();
        check(io.outputs.feederPower == 0 && io.outputs.shooterPower == 0
                && io.outputs.turnClockwise == 0, "lost target revokes latched feed permission");
        controller.stop();
    }

    private static void check(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
    }

    private static final class FakeIO implements RobotIO {
        double now, tx;
        boolean tv = true;
        int tagId;
        Pose pose;
        Outputs outputs = new Outputs();
        public double nowSeconds() { return now; }
        public Sensors readSensors() {
            Target[] targets = tv ? new Target[]{new Target(tagId, tx, 0, 70)} : new Target[0];
            return new Sensors(pose, now, true, true, -1,
                    new Vision(tv, 0, now, targets, tx));
        }
        public void apply(Outputs outputs) { this.outputs = outputs.copy(); }
    }
}
