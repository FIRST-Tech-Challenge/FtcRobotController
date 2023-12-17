package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

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
                new Constraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15),
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
    AutonDriveFactory(MecanumDrive drive) {
        this.drive = drive;
    }

    /*
     * Call this routine from your robot's competition code to get the sequence to drive. You
     * can invoke it there by calling "Actions.runBlocking(driveAction);".
     */
    Action getDriveAction(boolean isRed, boolean isFar) {
        TrajectoryActionBuilder build = this.drive.actionBuilder(new Pose2d(0, 0, 0));

        build = build.lineToX(30)
                .turn(Math.toRadians(90))
                .lineToY(30);

        build = build.splineTo(new Vector2d(0, 30), Math.toRadians(-90))
                .lineToY(0)
                .turn(Math.toRadians(90));

        return build.build();
    }

    /*
     * MeepMeep calls this routine to get a trajectory sequence action to draw. Modify the
     * arguments here to test your different code paths.
     */
    Action getMeepMeepAction() {
        return getDriveAction(true, false);
    }
}
