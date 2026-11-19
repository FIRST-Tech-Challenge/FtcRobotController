package org.firstinspires.ftc.teamcode.quantumstorm;

import org.firstinspires.ftc.teamcode.biobuzz.*;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

public class StateMachineTest {
    public static void main(String[] args) {
        Outputs outputs = new Outputs();
        ShootingAim.Alignment boundary = new ShootingAim.Alignment();
        check(!boundary.align(sample(true, 0, 10, 2), 10, outputs), "first centered frame does not permit shooting");
        check(!boundary.align(sample(true, 0, 10.05, 2), 10.05, outputs), "two centered frames do not permit shooting");
        check(boundary.align(sample(true, 0, 10.10, 2), 10.10, outputs), "three centered frames spanning 100ms permit shooting at the 2-degree boundary");
        ShootingAim.Alignment positiveTurn = new ShootingAim.Alignment();
        check(!positiveTurn.align(sample(true, 0, 10, 2.01), 10, outputs)
                && outputs.turnClockwise > 0, "positive tx turns clockwise");
        ShootingAim.Alignment negativeTurn = new ShootingAim.Alignment();
        check(!negativeTurn.align(sample(true, 0, 10, -30), 10, outputs)
                && outputs.turnClockwise == -.25, "negative tx turns counterclockwise with bounded power");
        check(!new ShootingAim.Alignment().align(sample(false, 0, 10, 0), 10, outputs)
                && outputs.turnClockwise == 0, "tv=false never turns or shoots");
        check(!new ShootingAim.Alignment().align(sample(true, 1, 10, 0), 10, outputs), "color pipeline cannot authorize shooting");
        ShootingAim.Alignment fresh = new ShootingAim.Alignment();
        fresh.align(sample(true, 0, 9.80, 0), 9.80, outputs);
        fresh.align(sample(true, 0, 9.85, 0), 9.85, outputs);
        check(fresh.align(sample(true, 0, 9.90, 0), 9.90, outputs)
                && fresh.align(sample(true, 0, 9.90, 0), 10.15, outputs), "250ms freshness boundary");
        check(!fresh.align(sample(true, 0, 9.90, 0), 10.151, outputs), "stale frame blocks shooting");
        check(!new ShootingAim.Alignment().align(sample(true, 0, 10, Double.NaN), 10, outputs), "nonfinite tx blocks shooting");
        check(ShootingAim.turnSuggestion(false, 30).startsWith("No turn"), "tv denies directional suggestions");
        check(ShootingAim.turnSuggestion(true, 12).contains("CLOCKWISE / RIGHT"), "positive tx suggests right");
        check(ShootingAim.turnSuggestion(true, -12).contains("COUNTERCLOCKWISE / LEFT"), "negative tx suggests left");
        check(ShootingAim.turnSuggestion(true, 2).startsWith("Aligned"), "aligned target suggests holding heading");
        check(ShootingAim.turnSuggestion(true, Double.NaN).startsWith("No turn"), "nonfinite tx has no suggestion");
        testHiveCenter();
        testAutonomous(false);
        testAutonomous(true);
        testColorPickup();
        testColorParking(false);
        testColorParking(true);
        System.out.println("Shooting aim and both autonomous gates passed");
    }

    private static void testHiveCenter() {
        FieldLayout field = new FieldLayout();
        Target[] four = {new Target(34, -12, 5), new Target(35, 12, 5),
                new Target(36, -12, -5), new Target(37, 12, -5), new Target(30, 40, 0)};
        Vision centered = ShootingAim.centerOnHive(new Vision(true, 0, 10, four, 30), field.nearTags, 0);
        Outputs out = new Outputs();
        out.forward = .5; out.strafeRight = .5;
        ShootingAim.Alignment centeredAlignment = new ShootingAim.Alignment();
        check(!centeredAlignment.align(new Sensors(new Pose(0, 0, 0), 10, true, true, -1, centered), 10, out)
                && !centeredAlignment.align(new Sensors(new Pose(0, 0, 0), 10.05, true, true, -1,
                        ShootingAim.centerOnHive(new Vision(true, 0, 10.05, four, 30), field.nearTags, 0)), 10.05, out)
                && centeredAlignment.align(new Sensors(new Pose(0, 0, 0), 10.10, true, true, -1,
                        ShootingAim.centerOnHive(new Vision(true, 0, 10.10, four, 30), field.nearTags, 0)), 10.10, out),
                "four tag centers override an off-axis aggregate tx and authorize after settling");
        check(out.forward == 0 && out.strafeRight == 0, "shooting alignment never translates");
        check(centered.targets.length == four.length && centered.capturedAt == 10,
                "centering preserves tag evidence and capture time for existing gates");
        Vision uneven = ShootingAim.centerOnHive(new Vision(true, 0, 10,
                new Target[]{new Target(34, 0, 0), new Target(35, 30, 0)}, 0), field.nearTags, 0);
        check(Math.abs(uneven.tx - Math.toDegrees(Math.atan(Math.tan(Math.toRadians(30)) / 2))) < 1e-9,
                "uneven angular spacing uses the image-plane centroid");

        Vision partial = ShootingAim.centerOnHive(new Vision(true, 0, 10,
                new Target[]{new Target(34, 12, 0)}, -30), field.nearTags, 0);
        check(!new ShootingAim.Alignment().align(new Sensors(new Pose(0, 0, 0), 10, true, true, -1, partial), 10, out)
                && out.turnClockwise == 0, "one visible tag is insufficient to authorize alignment");
        Vision partialPair = ShootingAim.centerOnHive(new Vision(true, 0, 10,
                new Target[]{new Target(34, 12, 0), new Target(35, 14, 0)}, -30), field.nearTags, 0);
        ShootingAim.Alignment partialAlignment = new ShootingAim.Alignment();
        check(!partialAlignment.align(new Sensors(new Pose(0, 0, 0), 10, true, true, -1, partialPair), 10, out)
                && out.turnClockwise > 0, "two visible correct tags steer toward the cluster");
        double[] wheels = out.wheelPowers();
        check(wheels[0] > 0 && wheels[2] > 0 && wheels[1] < 0 && wheels[3] < 0,
                "mecanum wheels counter-rotate for a clockwise in-place turn");
        ShootingAim.Alignment settle = new ShootingAim.Alignment();
        check(!settle.align(new Sensors(new Pose(0, 0, 0), 10, true, true, -1, centered), 10, out)
                && !settle.align(new Sensors(new Pose(0, 0, 0), 10, true, true, -1, centered), 10, out)
                && !settle.align(new Sensors(new Pose(0, 0, 0), 10.05, true, true, -1,
                        ShootingAim.centerOnHive(new Vision(true, 0, 10.05, four, 30), field.nearTags, 0)), 10.05, out)
                && !settle.align(new Sensors(new Pose(0, 0, 0), 10.05, true, true, -1,
                        ShootingAim.centerOnHive(new Vision(true, 0, 10.05, four, 30), field.nearTags, 0)), 10.05, out)
                && settle.align(new Sensors(new Pose(0, 0, 0), 10.10, true, true, -1,
                        ShootingAim.centerOnHive(new Vision(true, 0, 10.10, four, 30), field.nearTags, 0)), 10.10, out)
                && out.turnClockwise == 0, "duplicate frames do not count toward stable alignment");
        Vision offCenter = ShootingAim.centerOnHive(new Vision(true, 0, 10.15,
                new Target[]{new Target(34, 8, 0), new Target(35, 8, 0)}, 0), field.nearTags, 0);
        check(!settle.align(new Sensors(new Pose(0, 0, 0), 10.15, true, true, -1, offCenter), 10.15, out)
                && out.turnClockwise > 0, "drift outside the deadband restarts stable-frame qualification");
        check(!new ShootingAim.Alignment().align(new Sensors(new Pose(0, 0, 0), 10, false, true, -1, partialPair), 10, out)
                && out.turnClockwise == 0, "invalid odometry blocks cluster turning");
        check(!new ShootingAim.Alignment().align(new Sensors(new Pose(0, 0, 0), 10, true, true, -1, centered), 10.251, out),
                "cluster centering does not refresh stale evidence");
        Vision wrongCell = ShootingAim.centerOnHive(new Vision(true, 0, 10,
                new Target[]{new Target(30, 0, 0)}, 0), field.nearTags, 0);
        check(!new ShootingAim.Alignment().align(new Sensors(new Pose(0, 0, 0), 10, true, true, -1, wrongCell), 10, out),
                "unrelated centered tag cannot authorize shooting");
        Vision blue = ShootingAim.centerOnHive(new Vision(true, 0, 10,
                new Target[]{new Target(42, -10, 0)}, 30), field.nearTags, -1);
        check(Math.abs(blue.tx + 10) < 1e-9&&blue.aimTagCount==1, "robot-relative routes preserve the visible Blue shot cluster evidence");
        check(!new ShootingAim.Alignment().align(new Sensors(new Pose(0, 0, 0), 10, true, true, -1, blue), 10, out),
                "single-tag robot-relative cluster cannot authorize shooting");
        Vision ambiguous = ShootingAim.centerOnHive(new Vision(true, 0, 10,
                new Target[]{new Target(34, 10, 0), new Target(42, -10, 0)}, 0), field.nearTags, -1);
        check(Double.isNaN(ambiguous.tx), "two visible alliance clusters are not averaged together");
        Vision duplicate = ShootingAim.centerOnHive(new Vision(true, 0, 10,
                new Target[]{new Target(34, 10, 0), new Target(34, -10, 0)}, 0), field.nearTags, 0);
        check(Double.isNaN(duplicate.tx), "duplicate IDs cannot invent a centered cluster");
        Vision invalid = ShootingAim.centerOnHive(new Vision(true, 0, 10,
                new Target[]{new Target(34, Double.NaN, 0)}, 0), field.nearTags, 0);
        check(Double.isNaN(invalid.tx), "invalid expected-tag tx cannot fall back to aggregate tx");
        Vision far = ShootingAim.centerOnHive(new Vision(true, 0, 10,
                new Target[]{new Target(30, 10, 0), new Target(34, -30, 0)}, 0), field.farTags, 0);
        check(Math.abs(far.tx - 10) < 1e-9, "Team Two and follow-up shots use far-Cell IDs");
    }

    private static Sensors sample(boolean tv, int pipeline, double capturedAt, double tx) {
        Target[] targets = {new Target(34, tx, 0),new Target(35, tx, 0)};
        return new Sensors(new Pose(0, 0, 0), 10, true, true, -1,
                ShootingAim.centerOnHive(new Vision(tv, pipeline, capturedAt, targets, tx),new int[][]{{34,35}},0));
    }

    private static void testAutonomous(boolean teamTwo) {
        FieldLayout field = new FieldLayout();
        FakeIO io = new FakeIO();
        io.pose = teamTwo ? field.teamTwoWallHive[0] : field.hive[0];
        io.tagIds = teamTwo ? field.farTags[0] : field.nearTags[0];
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
        for (int i = 0; i < 4; i++) { io.now += .05; controller.tick(); }
        check(io.outputs.feederPower > 0, "aligned live target permits feed");
        io.tx = -12;
        io.now += .05;
        controller.tick();
        check(io.outputs.feederPower == 0 && io.outputs.shooterPower == 0
                && io.outputs.turnClockwise < 0, "alignment loss pauses feeding and corrects heading");
        check(io.outputs.forward == 0 && io.outputs.strafeRight == 0,
                "shooting correction turns in place without translating");
        double[] wheels = io.outputs.wheelPowers(-1);
        check(wheels[0] < 0 && wheels[2] < 0 && wheels[1] > 0 && wheels[3] > 0,
                "hardware forward correction preserves counterclockwise aiming");
        io.tv = false;
        io.now += .05;
        controller.tick();
        check(io.outputs.feederPower == 0 && io.outputs.shooterPower == 0
                && io.outputs.turnClockwise == 0, "lost target revokes latched feed permission");
        for (double wheel : io.outputs.wheelPowers(-1)) {
            check(wheel == 0, "lost target stops all four mecanum wheels");
        }
        controller.stop();
    }

    private static void check(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
    }

    private static Sensors colorSample(String color,double tx,double capturedAt,boolean odometry) {
        return new Sensors(new Pose(0,0,0),10,odometry,true,0,
                new Vision(true,1,capturedAt,new Target[]{new Target(-1,tx,0,Double.NaN,4,color)}));
    }

    private static Outputs pickupOutput() {
        Outputs out=new Outputs();out.forward=.18;out.intakePower=1;return out;
    }

    private static void testColorPickup() {
        Outputs out=pickupOutput();
        ColorPickup.steer(colorSample("yellow",10,10,true),10,out,true,false);
        check(out.turnClockwise==.20&&out.forward==.18,"yellow steers toward positive tx");
        out=pickupOutput();
        ColorPickup.steer(colorSample("blue",-10,10,true),10,out,true,false);
        check(out.turnClockwise==-.20,"blue=true steers toward blue");
        out=pickupOutput();
        ColorPickup.steer(colorSample("red",10,10,true),10,out,true,false);
        check(out.turnClockwise<0&&out.forward==0&&out.intakePower==0,"red=false stops collection and turns away");
        out=pickupOutput();
        ColorPickup.steer(colorSample("red",0,10,true),10,out,true,false);
        check(out.turnClockwise!=0&&out.forward==0,"centered red still has an escape turn");
        out=pickupOutput();
        ColorPickup.steer(colorSample("blue",10,9.749,true),10,out,true,false);
        check(out.turnClockwise==0&&out.forward==.18,"stale color cannot alter drive requests");
        ColorPickup.steer(colorSample("blue",10,10,false),10,out,true,false);
        check(out.turnClockwise==0,"invalid odometry prevents color turning");
        out=new Outputs();out.forward=.3;out.turnClockwise=.08;
        ColorPickup.steer(colorSample("red",10,10,true),10,out,true,false);
        check(out.forward==.3&&out.turnClockwise==.08,"intake off leaves parking drive untouched");
    }

    private static void testColorParking(boolean teamTwo) {
        FieldLayout field=new FieldLayout();
        final double[] now={0};
        final Outputs[] applied={new Outputs()};
        RobotIO io=new RobotIO(){
            public double nowSeconds(){return now[0];}
            public Sensors readSensors(){
                return new Sensors(teamTwo?field.teamTwoWallHive[0]:field.hive[0],now[0],true,true,0,
                        new Vision(true,1,now[0],new Target[]{new Target(-1,12,0,Double.NaN,4,"red")}));
            }
            public void apply(Outputs out){applied[0]=out.copy();}
        };
        RobotController controller;
        if(teamTwo){
            TeamTwoHiveAuto auto=new TeamTwoHiveAuto(io,field);auto.colorPickupEnabled=true;controller=auto;
        }else{
            TeamOnePollenAuto auto=new TeamOnePollenAuto(io,field);auto.colorPickupEnabled=true;controller=auto;
        }
        controller.start(0,.54);
        now[0]=26;controller.tick();
        check(controller.getState().toString().equals("PARK")&&applied[0].intakePower==0,
                "26-second parking overrides red avoidance and collection");
        check(teamTwo?!((TeamTwoHiveAuto)controller).colorPickupActive():!((TeamOnePollenAuto)controller).colorPickupActive(),
                "color steering is disabled during parking");
        now[0]=30;controller.tick();
        for(double wheel:applied[0].wheelPowers())check(wheel==0,"30-second stop turns every wheel off");
        check(applied[0].intakePower==0&&applied[0].shooterPower==0&&applied[0].feederPower==0,
                "30-second stop keeps mechanisms off");
    }

    private static final class FakeIO implements RobotIO {
        double now, tx;
        boolean tv = true;
        int[] tagIds;
        Pose pose;
        Outputs outputs = new Outputs();
        public double nowSeconds() { return now; }
        public Sensors readSensors() {
            Target[] targets = new Target[tv?tagIds.length:0];
            if(tv)for(int i=0;i<tagIds.length;i++)targets[i]=new Target(tagIds[i],tx,0,70);
            return new Sensors(pose, now, true, true, -1,
                    ShootingAim.centerOnHive(new Vision(tv, 0, now, targets, -tx),
                            new int[][]{tagIds}, 0));
        }
        public void apply(Outputs outputs) { this.outputs = outputs.copy(); }
    }
}
