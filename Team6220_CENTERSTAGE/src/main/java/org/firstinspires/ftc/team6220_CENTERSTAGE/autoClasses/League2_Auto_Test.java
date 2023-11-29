package org.firstinspires.ftc.team6220_CENTERSTAGE.autoClasses;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_CENTERSTAGE.MecanumDrive;

@Autonomous(name="League2_Auto_Test", group ="amogus2")
public class League2_Auto_Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonDriveFactory autoDrive = new AutonDriveFactory(new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0)));



        waitForStart();

        Action driveAction = autoDrive.getDriveAction(AutonDriveFactory.AllianceTeam.BLUE, AutonDriveFactory.StartingPosition.SHORT, AutonDriveFactory.ParkLocation.CORNER_SIDE);
        Actions.runBlocking(driveAction);
    }

}
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

        this.drive.pose = startingPose; // DISABLE THIS WHEN DOING MEEPMEEP!!

        // start building a trajectory path
        TrajectoryActionBuilder build = this.drive.actionBuilder(startingPose);

        // drive backwards
        build = build.lineToY(36 * invertSign)
                .endTrajectory();

        // purple pixel placement
        build = build.waitSeconds(1); // PLACEHOLDER

        // return to starting position while turning to face away from backdrop
        /*
        build = build.lineToY(55 * invertSign)
                .lineToYLinearHeading(59 * invertSign, Math.toRadians(180))
                .endTrajectory();

         */
        build = build.lineToY(55 * invertSign)
                .turnTo(Math.toRadians(180))
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
