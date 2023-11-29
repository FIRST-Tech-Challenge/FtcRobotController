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
                18, 18,
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

    // used in determining which alliance side to use in auto path
    public enum AllianceTeam {
        BLUE,
        RED,
    }
    // used in determining which side of the truss to use in auto path
    // name represents path length to backstage
    public enum StartingPosition {
        SHORT,
        LONG,
    }
    // determines where on the backstage to park
    public enum ParkLocation {
        CORNER_SIDE,
        CENTER_FIELD_SIDE,
    }

    MecanumDrive drive;
    AutonDriveFactory(MecanumDrive drive) {
        this.drive = drive;
    }

    /*
     * Call this routine from your robot's competition code to get the sequence to drive. You
     * can invoke it there by calling "Actions.runBlocking(driveAction);".
     */
    Action getDriveAction(AllianceTeam allianceTeam, StartingPosition startingPosition, ParkLocation parkLocation) {

        // use 1 if blue team, -1 if red
        int invertSign = allianceTeam == AllianceTeam.BLUE ? 1 : -1;

        // determine the starting pose
        Pose2d startingPose = new Pose2d(0,0,0);
        switch (startingPosition) {
            case SHORT:

                startingPose = new Pose2d (11.6, 61 * invertSign, Math.toRadians(90 * invertSign));
                break;

            case LONG:

                startingPose = new Pose2d (-35, 61 * invertSign, Math.toRadians(90 * invertSign));
                break;
        }

        // start building a trajectory path
        TrajectoryActionBuilder build = this.drive.actionBuilder(startingPose);

        // drive backwards
        build = build.lineToY(36 * invertSign)
                     .endTrajectory();

        // purple pixel placement
        build = build.waitSeconds(1); // PLACEHOLDER

        // return to starting position while turning to face away from backdrop
        build = build.lineToY(55 * invertSign)
                     .lineToYLinearHeading(59 * invertSign, Math.toRadians(180))
                     .endTrajectory();

        // drive towards the backdrop
        if (startingPosition == StartingPosition.LONG) {
            // wait
            build = build.waitSeconds(1); // temp wait time
        }
        build = build.lineToX(20);

        // go to face backdrop
        build = build.splineToLinearHeading(new Pose2d(48, 36 * invertSign, Math.toRadians(180)), Math.toRadians(0));

        // place yellow pixel
        build = build.waitSeconds(1); // PLACEHOLDER

        // drive in front of park location
        switch (parkLocation) {
            case CORNER_SIDE:
                // strafe to corner side
                build = build.setTangent(Math.toRadians(90 * invertSign))
                             .lineToY(59 * invertSign);
                break;

            case CENTER_FIELD_SIDE:
                // strafe to center field side
                build = build.setTangent(Math.toRadians(-90 * invertSign))
                             .lineToY(12 * invertSign);
                break;
        }

        // drive backwards into backstage
        build = build.setTangent(Math.toRadians(0))
                .lineToX(60);

        // finish build
        return build.build();
    }

    /*
     * MeepMeep calls this routine to get a trajectory sequence action to draw. Modify the
     * arguments here to test your different code paths.
     */
    Action getMeepMeepAction() {
        return getDriveAction(AllianceTeam.BLUE, StartingPosition.SHORT, ParkLocation.CORNER_SIDE);
    }
}
