package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

/**
 * Wrapper class for MeepMeep testing. Modify the 'myBot' constructor settings to reflect your
 * own robot. You shouldn't need to modify any other code here.
 */
public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new RoadRunnerBotEntity(
                meepMeep,
                // Robot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width:
                new Constraints(30, 60, Math.toRadians(180), Math.toRadians(180), 15),
                // Robot dimensions: width, height:
                15, 15,
                new Pose2d(0, 0, 0),
                meepMeep.getColorManager().getTheme(),
                0.8f,
                DriveTrainType.MECANUM,
                false);

        MecanumDrive drive = new MecanumDrive(myBot.getDrive());
        AutonDriveFactory auton = new AutonDriveFactory(drive);

        myBot.runAction(auton.getMeepMeepAction());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

/**
 * Shim so that the AutonDriveFactory can refer to the drive using a MecanumDrive type both
 * here in MeepMeep and also in the competition code.
 */
class MecanumDrive {
    DriveShim shim;
    MecanumDrive(DriveShim shim) {
        this.shim = shim;
    }
    TrajectoryActionBuilder actionBuilder(Pose2d pose) {
        return this.shim.actionBuilder(pose);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * This is a template for your autonomous code. It's called a "factory" because it creates objects
 * (Road Runner 'Action' objects in this case). Feel free to rename it to whatever you
 * prefer. By making this class independent of all of your other classes, and independent of any
 * FTC code, it can let you ^C copy this entire class back and forth between your competition
 * code and this MeepMeep test code. But if you add a dependency on something like DcMotor (which
 * is FTC code), it will stop compiling in MeepMeep and you won't be able to test your logic
 * anymore.
 */
class AutonDriveFactory {
    MecanumDrive drive;
    double xOffset;
    double yMultiplier;
    AutonDriveFactory(MecanumDrive drive) {
        this.drive = drive;
    }

    /*
     * Call this routine from your robot's competition code to get the sequence to drive. You
     * can invoke it there by calling "Actions.runBlocking(driveAction);".
     */
    enum SpikeMarks {
        LEFT,
        CENTER,
        RIGHT
    }

    class PoseAndAction {
        Action action;
        Pose2d startPose;

        PoseAndAction(Action action, Pose2d startPose) {
            this.action = action;
            this.startPose = startPose;
        }
    }
    PoseAndAction getDriveAction(boolean isRed, boolean isFar, SpikeMarks location, Action intake) {

        if (isFar) {
            xOffset = 0;
        } else {
            xOffset = 48;
        }

        if (isRed) {
            yMultiplier = 1;
        } else {
            yMultiplier = -1;
        }

        // in MeepMeep, intake needs to be null however .stopAndAdd() can't be null because it will crash so we set to a random sleep
        if(intake == null) {
            intake = new SleepAction(3);
        }

        TrajectoryActionBuilder spikeLeft = this.drive.actionBuilder(xForm(new Pose2d(-34, -60, Math.toRadians(90))));
        spikeLeft = spikeLeft.splineTo(xForm(new Vector2d(-34, -36)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-38, -34)), xForm(Math.toRadians(180) + (1e-6)))
                .stopAndAdd(intake)
                .splineToConstantHeading(xForm(new Vector2d(-30, -34)), xForm(Math.toRadians(180)))
                .splineTo(xForm(new Vector2d(-34, -30)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-30, -10)), xForm(Math.toRadians(0)))
                .splineToConstantHeading(xForm(new Vector2d(58, -10)), xForm(Math.toRadians(0)));

        TrajectoryActionBuilder spikeCenter = this.drive.actionBuilder(xForm(new Pose2d(-34, -60, Math.toRadians(90))));
        spikeCenter = spikeCenter.splineTo(xForm(new Vector2d(-34, -33)), xForm(Math.toRadians(90)))
                // arm
                .splineToConstantHeading(xForm(new Vector2d(-34, -39)), xForm(Math.toRadians(90)))
                .splineToConstantHeading(xForm(new Vector2d(-55, -39)), xForm(Math.toRadians(90)))
                .splineToConstantHeading(xForm(new Vector2d(-55, -10)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-30, -10)), xForm(Math.toRadians(0)))
                .splineToConstantHeading(xForm(new Vector2d(58, -10)), xForm(Math.toRadians(0)));


        TrajectoryActionBuilder spikeRight = this.drive.actionBuilder(xForm(new Pose2d(-34, -60, Math.toRadians(90))));
        spikeRight = spikeRight.splineToSplineHeading(xForm(new Pose2d(-35, -32, Math.toRadians(0))), xForm(Math.toRadians(90)))
                // arm action
                .splineToConstantHeading(xForm(new Vector2d(-40, -34)), xForm(Math.toRadians(0)))
                .splineTo(xForm(new Vector2d(-36, -30)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-30, -10)), xForm(Math.toRadians(0)))
                .splineToConstantHeading(xForm(new Vector2d(58, -10)), xForm(Math.toRadians(0)));

        if(location == SpikeMarks.LEFT) {
            return new PoseAndAction(spikeLeft.build(), xForm(new Pose2d(-34, -60, Math.toRadians(90))));
        } else if(location == SpikeMarks.CENTER) {
            return new PoseAndAction(spikeCenter.build(), xForm(new Pose2d(-34, -60, Math.toRadians(90))));
        } else {
            return new PoseAndAction(spikeRight.build(), xForm(new Pose2d(-34, -60, Math.toRadians(90))));
        }

    }


    Pose2d xForm(Pose2d pose) {
        return new Pose2d(pose.position.x + xOffset, pose.position.y * yMultiplier, pose.heading.log() * yMultiplier);
    }

    Vector2d xForm(Vector2d vector) {
        return new Vector2d(vector.x + xOffset, vector.y * yMultiplier);
    }

    double xForm(double angle) {
        return (angle * yMultiplier);
    }


    /*
     * MeepMeep calls this routine to get a trajectory sequence action to draw. Modify the
     * arguments here to test your different code paths.
     */
    Action getMeepMeepAction() {
        return getDriveAction(true, true, SpikeMarks.LEFT, null).action;
    }
}
