package org.firstinspires.ftc.team6220_CENTERSTAGE.autoClasses;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_CENTERSTAGE.ColorDetection;
import org.firstinspires.ftc.team6220_CENTERSTAGE.Constants;
import org.firstinspires.ftc.team6220_CENTERSTAGE.MecanumDrive;

@Autonomous(name="League2_Auto_Test", group ="amogus2")
public class League2_Auto_Test extends LinearOpMode {
    ColorDetection colorDetector = new ColorDetection(this);
    @Override
    public void runOpMode() throws InterruptedException {
        AutonDriveFactory autoDrive = new AutonDriveFactory(new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0)));

        AutoParams params = new AutoParams();
        params.allianceTeam = AutoParams.AllianceTeam.BLUE;
        params.startingPosition = AutoParams.StartingPosition.SHORT;

        params.parkLocation = AutoParams.ParkLocation.CORNER_SIDE;

        colorDetector.init(Constants.BLUE_COLOR_DETECT_MIN_HSV, Constants.BLUE_COLOR_DETECT_MAX_HSV);
        //colorDetector.init(Constants.RED_COLOR_DETECT_MIN_HSV, Constants.RED_COLOR_DETECT_MAX_HSV);

        waitForStart();

        params.propPosition = colorDetector.returnZone();
        telemetry.addData("pos",params.propPosition);
        telemetry.update();

        Action driveAction = autoDrive.getDriveAction(params);
        Actions.runBlocking(driveAction);
    }

}

class AutonDriveFactory {

    MecanumDrive drive;
    AutonDriveFactory(MecanumDrive drive) {
        this.drive = drive;
    }

    /*
     * Call this routine from your robot's competition code to get the sequence to drive. You
     * can invoke it there by calling "Actions.runBlocking(driveAction);".
     */
    Action getDriveAction(AutoParams params) {

        // use 1 if blue team, -1 if red
        int invertSign = params.allianceTeam == AutoParams.AllianceTeam.BLUE ? 1 : -1;

        // determine the starting pose
        Pose2d startingPose = new Pose2d(0,0,0);
        switch (params.startingPosition) {
            case SHORT:

                startingPose = new Pose2d (11.6, 61 * invertSign, Math.toRadians(90 * invertSign));
                break;

            case LONG:

                startingPose = new Pose2d (-35, 61 * invertSign, Math.toRadians(90 * invertSign));
                break;
        }

        this.drive.pose = startingPose;

        // start building a trajectory path
        TrajectoryActionBuilder build = this.drive.actionBuilder(startingPose);

        // drive backwards
        build = build.lineToY(36 * invertSign)
                .endTrajectory();

        // purple pixel placement
        switch (params.propPosition) {
            case LEFT:
                build = build.turnTo(Math.toRadians(90 + 90 * invertSign));
                build = build.waitSeconds(1); // PLACEHOLDER
                build = build.turnTo(Math.toRadians(90 * invertSign));
                break;

            case MIDDLE:
                build = build.turnTo(Math.toRadians(90 * invertSign));
                build = build.waitSeconds(1); // PLACEHOLDER
                break;

            case RIGHT:
                build = build.turnTo(Math.toRadians(90 - 90 * invertSign));
                build = build.waitSeconds(1); // PLACEHOLDER
                build = build.turnTo(Math.toRadians(90 * invertSign));
                break;
        }

        // return to starting position while turning to face away from backdrop
        build = build.lineToY(55 * invertSign)
                .turnTo(Math.toRadians(180))
                .endTrajectory();

        // drive towards the backdrop
        if (params.startingPosition == AutoParams.StartingPosition.LONG) {
            // wait
            build = build.waitSeconds(1); // temp wait time
        }
        build = build.lineToX(20);

        // go to face backdrop
        build = build.splineToLinearHeading(new Pose2d(48, 36 * invertSign, Math.toRadians(180)), Math.toRadians(0));

        // place yellow pixel
        build = build.waitSeconds(1); // PLACEHOLDER

        // drive in front of park location
        switch (params.parkLocation) {
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

        AutoParams params = new AutoParams();
        params.allianceTeam = AutoParams.AllianceTeam.RED;
        params.startingPosition = AutoParams.StartingPosition.SHORT;
        params.parkLocation = AutoParams.ParkLocation.CORNER_SIDE;
        params.propPosition = ColorDetection.PropPosition.LEFT;

        return getDriveAction(params);
    }
}

class AutoParams {

    // used in determining which alliance side to use in auto path
    public static enum AllianceTeam {
        BLUE,
        RED,
    }
    // used in determining which side of the truss to use in auto path
    // name represents path length to backstage
    public static enum StartingPosition {
        SHORT,
        LONG,
    }
    // determines where on the backstage to park
    public static enum ParkLocation {
        CORNER_SIDE,
        CENTER_FIELD_SIDE,
    }

    public AllianceTeam allianceTeam;
    public StartingPosition startingPosition;
    public ParkLocation parkLocation;
    public ColorDetection.PropPosition propPosition;

    public AutoParams() {}

}