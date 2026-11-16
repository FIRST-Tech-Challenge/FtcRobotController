package org.firstinspires.ftc.teamcode.methods;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

public final class AutoPaths {
    private AutoPaths() {}
    private static final PoseFactory poseFactory = PoseFactory.degrees();

    public static final class GeneralPoints {
        private GeneralPoints() {}

        // close (bottom) side
        public static final Pose startClose =
                poseFactory.of(56, 8, 90);

        public static final Pose shootClose =
                poseFactory.of(56, 12, 90);

        public static final Pose flowerWaitClose =
                poseFactory.of(33, 46.5, 180);

        public static final Pose flowerClose =
                poseFactory.of(12, 48.5, 180);

        public static final Pose jiggleC =
                poseFactory.of(11.7, 48.5, 170);

        public static final Pose cornerCollect =
                poseFactory.of(9, 8.5, 180);

        public static final Pose parkCorner =
                poseFactory.of(12, 10, 180);

        public static final Pose parkUp =
                poseFactory.of(12, 94, 270);

        // far (top) side
        public static final Pose startFar =
                poseFactory.of(59, 135, 270);

        public static final Pose shootFar =
                poseFactory.of(57, 130, 270);

        public static final Pose farFlowerVia =
                poseFactory.of(60, 130, 270);

        public static final Pose flowerWaitFar =
                poseFactory.of(47, 120, 80);

        public static final Pose flowerFar =
                poseFactory.of(47, 128.5, 90);

        public static final Pose jiggle =
                poseFactory.of(47, 128.3, 90);

        public static final Pose midBack =
                poseFactory.of(47, 125, 90);

        public static final Pose parkFar =
                poseFactory.of(26, 130, 90);
    }

    // GENERAL PATHS (shared by more than one auto)
    public static final class GeneralPaths {

        private GeneralPaths() {}

        // start -> close shooting spot (first move of every close auto)
        public static Path startToShootClose() {
            return Paths.line(GeneralPoints.startClose, GeneralPoints.shootClose)
                    .linear(GeneralPoints.startClose, GeneralPoints.shootClose);
        }

        // close shooting spot -> close flower wait spot
        public static Path shootCloseToFlowerWait() {
            return Paths.line(GeneralPoints.shootClose, GeneralPoints.flowerWaitClose)
                    .linear(GeneralPoints.shootClose, GeneralPoints.flowerWaitClose);
        }

        public static Path flowerCloseTo() {
            return Paths.line(GeneralPoints.flowerWaitClose, GeneralPoints.flowerClose)
                    .linear(GeneralPoints.flowerWaitClose, GeneralPoints.flowerClose);
        }

        public static Path flowerCloseFro() {
            return Paths.line(GeneralPoints.flowerClose, GeneralPoints.flowerWaitClose)
                    .linear(GeneralPoints.flowerClose, GeneralPoints.flowerWaitClose);
        }

        public static Path flowerFarTo() {
            return Paths.line(GeneralPoints.flowerWaitFar, GeneralPoints.flowerFar)
                    .linear(GeneralPoints.flowerWaitFar, GeneralPoints.flowerFar);
        }

        public static Path flowerFarFro() {
            return Paths.line(GeneralPoints.flowerFar, GeneralPoints.flowerWaitFar)
                    .linear(GeneralPoints.flowerFar, GeneralPoints.flowerWaitFar);
        }

        public static Path flowerJiggle() {
            return Paths.line(GeneralPoints.flowerFar, GeneralPoints.jiggle)
                    .linear(GeneralPoints.flowerFar, GeneralPoints.jiggle);
        }

        public static Path jiggleFlower() {
            return Paths.line(GeneralPoints.jiggle, GeneralPoints.flowerFar)
                    .linear(GeneralPoints.jiggle, GeneralPoints.flowerFar);
        }

        public static Path halfJiggle() {
            return Paths.line(GeneralPoints.flowerFar, GeneralPoints.midBack)
                    .linear(GeneralPoints.flowerFar, GeneralPoints.midBack);
        }



        public static Path flowerJiggleC() {
            return Paths.line(GeneralPoints.flowerClose, GeneralPoints.jiggleC)
                    .linear(GeneralPoints.flowerClose, GeneralPoints.jiggleC);
        }

        public static Path jiggleFlowerC() {
            return Paths.line(GeneralPoints.jiggleC, GeneralPoints.flowerClose)
                    .linear(GeneralPoints.jiggleC, GeneralPoints.flowerClose);
        }
    }

    // TIP (2Tip)
    public static final class Tip {

        private Tip() {}

        // ---------- POSES ----------

        // corner -> far shoot curve: runs up the x~32 lane, left of the hive
        public static final Pose farControl1 =
                poseFactory.of(51, 28, 0);

        public static final Pose farControl2 =
                poseFactory.of(36, 63, 0);

        public static final Pose farControl3 =
                poseFactory.of(6, 90, 0);

        public static final Pose farControl4 =
                poseFactory.of(49, 112, 0);

        public static final Pose park =
                poseFactory.of(10, 122, 90);

        public static final Pose parkControl =
                poseFactory.of(5, 135, 0);

        // ---------- PATHS ----------

        // shoot spot -> corner in one move (was path2wall + path21).
        // Same straight lane along the wall; the 90->180 turn is done by 55%,
        // which is where the old (30, 11) stop was.
        public static Path shootCloseToCorner() {
            return Paths.line(GeneralPoints.shootClose, GeneralPoints.cornerCollect)
                    .linear(GeneralPoints.shootClose, GeneralPoints.cornerCollect, 0.55);
        }

        // corner -> far shoot spot in one move (was path22 + path3).
        // Holds 180 until the robot is ~30" off both walls, then turns to 270.
        public static Path cornerToFarShoot() {
            return Paths.curve(
                    GeneralPoints.cornerCollect,
                    farControl1,
                    farControl2,
                    farControl3,
                    farControl4,
                    GeneralPoints.farFlowerVia
            ).heading(Interpolator.piecewise()
                    .until(0.2, Interpolator.constant(GeneralPoints.cornerCollect))
                    .until(1.0, Interpolator.linear(GeneralPoints.cornerCollect, GeneralPoints.farFlowerVia)));
        }

        // far shoot spot -> far flower wait (was path4, which had a hairpin)
        public static Path farShootToFlowerWait() {
            return Paths.line(GeneralPoints.farFlowerVia, GeneralPoints.flowerWaitFar)
                    .linear(GeneralPoints.farFlowerVia, GeneralPoints.flowerWaitFar);
        }

        // far flower wait -> park (was path5, unchanged)
        public static Path park() {
            return Paths.curve(
                    GeneralPoints.flowerWaitFar,
                    parkControl,
                    park
            ).linear(GeneralPoints.flowerWaitFar, park);
        }
    }

    // CORNER (1Tip)
    public static final class Corner {

        private Corner() {}

        public static Path park() {
            return Paths.line(GeneralPoints.shootClose, GeneralPoints.parkCorner)
                    .linear(GeneralPoints.shootClose, GeneralPoints.parkCorner);
        }
    }

    // FLOWER (1Tip + Flower)
    public static final class Flower {

        private Flower() {}

        public static Path park() {
            return Paths.line(GeneralPoints.flowerWaitClose, GeneralPoints.parkCorner)
                    .linear(GeneralPoints.flowerWaitClose, GeneralPoints.parkCorner);
        }
    }

    // 3Tip
    public static final class TweaTip {

        private TweaTip() {}

        // ---------- POSES ----------

        public static final Pose shoot =
                poseFactory.of(36, 12, 180);

        public static final Pose park =
                poseFactory.of(10, 90, 270);

        // ---------- PATHS ----------

        public static Path flowerWaitToShoot() {
            return Paths.line(GeneralPoints.flowerWaitClose, shoot)
                    .linear(GeneralPoints.flowerWaitClose, shoot);
        }

        public static Path WallTo() {
            return Paths.line(shoot, GeneralPoints.cornerCollect)
                    .linear(shoot, GeneralPoints.cornerCollect);
        }

        public static Path WallFro() {
            return Paths.line(GeneralPoints.cornerCollect, shoot)
                    .linear(GeneralPoints.cornerCollect, shoot);
        }

        public static Path park() {
            return Paths.line(shoot, park)
                    .linear(shoot, park);
        }
    }

    // CORNERUP (1Tip + Park)
    public static final class CornerUp {

        private CornerUp() {}

        public static final Pose parkControl =
                poseFactory.of(20, 37, 0);

        public static Path park() {
            return Paths.curve(
                    GeneralPoints.shootClose,
                    parkControl,
                    GeneralPoints.parkUp
            ).linear(GeneralPoints.shootClose, GeneralPoints.parkUp);
        }
    }

    // FLOWERUP (1Tip + Flower + Park)
    public static final class FlowerUp {

        private FlowerUp() {}

        public static Path park() {
            return Paths.line(GeneralPoints.flowerWaitClose, GeneralPoints.parkUp)
                    .linear(GeneralPoints.flowerWaitClose, GeneralPoints.parkUp);
        }
    }

    // OPP (1TipFar)
    public static final class Opp {

        private Opp() {}

        // ---------- POSES ----------

        public static final Pose collect =
                poseFactory.of(26, 126, 90);

        public static final Pose park =
                poseFactory.of(15, 113, 90);

        // ---------- PATHS ----------

        public static Path startToShoot() {
            return Paths.line(GeneralPoints.startFar, GeneralPoints.shootFar)
                    .linear(GeneralPoints.startFar, GeneralPoints.shootFar);
        }

        // far shoot spot -> far flower wait (was path3, which had a hairpin)
        public static Path shootToFlowerWait() {
            return Paths.line(GeneralPoints.shootFar, GeneralPoints.flowerWaitFar)
                    .linear(GeneralPoints.shootFar, GeneralPoints.flowerWaitFar);
        }

        public static Path flowerWaitToCollect() {
            return Paths.line(GeneralPoints.flowerWaitFar, collect)
                    .linear(GeneralPoints.flowerWaitFar, collect);
        }

        public static Path collectCreep() {
            return Paths.line(collect, GeneralPoints.parkFar)
                    .linear(collect, GeneralPoints.parkFar);
        }

        public static Path park() {
            return Paths.line(GeneralPoints.parkFar, park)
                    .linear(GeneralPoints.parkFar, park);
        }
    }
}
