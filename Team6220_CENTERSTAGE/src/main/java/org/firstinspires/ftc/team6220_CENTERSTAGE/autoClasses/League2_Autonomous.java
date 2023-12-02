package org.firstinspires.ftc.team6220_CENTERSTAGE.autoClasses;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_CENTERSTAGE.ColorDetection;
import org.firstinspires.ftc.team6220_CENTERSTAGE.Constants;
import org.firstinspires.ftc.team6220_CENTERSTAGE.MecanumDrive;
import org.jetbrains.annotations.NotNull;

import org.firstinspires.ftc.team6220_CENTERSTAGE.JavaTextMenu.*;

@Autonomous(name="League2_Autonomous", group ="amogus2")
public class League2_Autonomous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        AutonDriveFactory autoDrive = new AutonDriveFactory(new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0)));

        // setup menu
        TextMenu menu = new TextMenu();
        MenuInput menuInput = new MenuInput(MenuInput.InputType.CONTROLLER);

        menu.add("League 2 Autonomous Options")
                .add()
                .add("Alliance team:")
                .add("select-team", AutoParams.AllianceTeam.class)
                .add()
                .add("Starting position:")
                .add("start-select", AutoParams.StartingPosition.class)
                .add()
                .add("Park destination:")
                .add("park-select", AutoParams.ParkLocation.class)
                .add()
                .add("Start long wait time:")
                .add("long-wait", new MenuSlider(0, 20, 1, 10))
                .add()
                .add("Place purple pixel?")
                .add("purple-switch", new MenuSwitch(true))
                .add()
                .add("Place yellow pixel?")
                .add("yellow-switch", new MenuSwitch(true))
                .add()
                .add("finish", new MenuFinishedButton())
        ;

        // let user access menu
        while (!menu.isCompleted()) {
            // print menu
            for (String line : menu.toListOfStrings()) {
                telemetry.addLine(line);
            }
            telemetry.update();

            // update with input
            menuInput.update(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.a);
            menu.updateWithInput(menuInput);
        }


        // setup auto parameters using menu results
        AutoParams params = new AutoParams();
        params.allianceTeam = menu.getResult("select-team", AutoParams.AllianceTeam.class);
        params.startingPosition = menu.getResult("start-select", AutoParams.StartingPosition.class);
        params.parkLocation = menu.getResult("park-select", AutoParams.ParkLocation.class);
        //params.propPosition = ColorDetection.PropPosition.LEFT; // we get it after wait for start
        params.startLongWaitTime = menu.getResult("long-wait", Double.class);
        params.placePurplePixel = menu.getResult("purple-switch", Boolean.class);
        params.placeYellowPixel = menu.getResult("yellow-switch", Boolean.class);

        ColorDetection colorDetector = new ColorDetection(this);
        if (!autoDrive.drive.isDevBot) {
            // init color detector after determining which alliance we're on
            switch (params.allianceTeam) {

                case BLUE:
                    colorDetector.init(Constants.BLUE_COLOR_DETECT_MIN_HSV, Constants.BLUE_COLOR_DETECT_MAX_HSV);
                    break;

                case RED:
                    colorDetector.init(Constants.RED_COLOR_DETECT_MIN_HSV, Constants.RED_COLOR_DETECT_MAX_HSV);
                    break;
            }
        }

        // wait until we hit start auto
        telemetry.addLine("Waiting for start...");
        telemetry.addLine();
        telemetry.addData("Team:", params.allianceTeam);
        telemetry.addData("Start:", params.startingPosition);
        telemetry.addData("Park:", params.parkLocation);
        telemetry.addData("Start long wait:", params.startLongWaitTime);
        telemetry.addData("Place purple:", params.placePurplePixel);
        telemetry.addData("Place purple:", params.placeYellowPixel);
        telemetry.update();

        waitForStart();

        if (!autoDrive.drive.isDevBot) {
            // capture the position of the prop
            params.propPosition = colorDetector.returnZone();
        } else {
            telemetry.addLine("isDevBot: choosing random prop position");
            ColorDetection.PropPosition[] propPositions = {ColorDetection.PropPosition.RIGHT, ColorDetection.PropPosition.MIDDLE, ColorDetection.PropPosition.LEFT};
            params.propPosition = propPositions[(int)Math.floor(Math.random()*3)];
        }

        telemetry.addData("Captured prop position:", params.propPosition);
        telemetry.update();

        // run the path with the chosen parameters
        Action driveAction = autoDrive.getDriveAction(params);
        Actions.runBlocking(driveAction);
    }

}

//////////////////////////////////////////////////////////////////////////////

class AutonDriveFactory {

    private enum SpikeType {
        OPEN, // side spike that doesn't have truss around it
        MIDDLE, // spike that is in the middle
        TRUSS, // side spike that has truss around it
    }

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
        int teamInvert = 0;
        switch (params.allianceTeam) {

            case BLUE:
                teamInvert = 1;
                break;

            case RED:
                teamInvert = -1;
                break;
        }

        // determine the starting pose
        int startingPosX = 0;
        int startInvert = 0;
        switch (params.startingPosition) {

            case SHORT:
                startingPosX = 12;
                startInvert = 1;
                break;

            case LONG:
                startingPosX = -36;
                startInvert = -1;
                break;
        }
        Pose2d startingPose = new Pose2d(startingPosX, 63 * teamInvert, Math.toRadians(-90 * teamInvert));

        // apply to drive so it doesn't think it's starting at (0,0,0)
        this.drive.pose = startingPose;

        // start building a trajectory path
        TrajectoryActionBuilder build = this.drive.actionBuilder(startingPose);

        if (params.placePurplePixel) {

            /*
            we need to convert the raw left/middle/right info from prop detector
            into a spike type that has the correct corresponding path
            but that depends on starting position, team and prop pos:

            HOW THE SPIKE TYPE SECTION BELOW WORKS

            (middle is always middle spike)

            SHORT   BLUE    LEFT    RIGHT
            true    true    open    truss
            true    false   truss   open
            false   true    truss   open
            false   false   open    truss

            replace short and blue with invert values of 1 and -1:

            SHORT   BLUE    SUMMED  LEFT    RIGHT
            1       1       2       open    truss
            1       -1      0       truss   open
            -1      1       0       truss   open
            -1      -1      -2      open    truss

            so if sum is 0 then left->truss and right->open
            otherwise left->open and right->truss
            and middle is always middle so only apply if it's left or right
             */

            SpikeType spikeType = SpikeType.MIDDLE; // default to middle

            // refer to block comment above
            int invertSum = teamInvert + startInvert;
            if (invertSum == 0) {
                switch (params.propPosition) {
                    case LEFT:
                        spikeType = SpikeType.TRUSS;
                        break;
                    case RIGHT:
                        spikeType = SpikeType.OPEN;
                        break;
                }
            } else {
                switch (params.propPosition) {
                    case LEFT:
                        spikeType = SpikeType.OPEN;
                        break;
                    case RIGHT:
                        spikeType = SpikeType.TRUSS;
                        break;
                }
            }

            // go to prop position spike mark using correct spike path
            switch (spikeType) {

                case OPEN:
                    build = build.strafeTo(new Vector2d(startingPosX + 11 * startInvert, 40 * teamInvert));
                    break;

                case MIDDLE:
                    build = build.lineToY(34 * teamInvert);
                    break;

                case TRUSS:
                    build = build.lineToY(36 * teamInvert)
                            .turnTo(Math.toRadians(90 + 90 * startInvert))
                            .lineToX(startingPosX - 2 * startInvert);
                    break;
            }

            // place purple pixel
            build = build.waitSeconds(1); // PLACEHOLDER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

            // return to the side of the field so that we can go towards the backdrop
            switch (spikeType) {

                case OPEN:
                case MIDDLE:
                    build = build.lineToY(60 * teamInvert)
                            .turnTo(Math.toRadians(180));
                    break;

                case TRUSS:
                    build = build.lineToX(startingPosX)
                            .strafeTo(new Vector2d(startingPosX, 60 * teamInvert));
                    break;
            }
            // end of params.placePurplePixel
        } else {
            build = build.lineToY(60 * teamInvert);
        }

        // turn to point outtake towards backdrop
        build = build.turnTo(Math.toRadians(180));

        // get to known spot before spline to backdrop
        switch (params.startingPosition) {
            case LONG:
                // wait for alliance to move out of the way
                build = build.waitSeconds(params.startLongWaitTime);
                break;
        }

        // towards backdrop side
        build = build.setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(24, 60 * teamInvert), Math.toRadians(0), limitVelo(40));

        if (params.placeYellowPixel) {

            // drive to face backdrop (move slowly when close)
            build = build.splineToConstantHeading(new Vector2d(46, 36 * teamInvert), Math.toRadians(0), limitVelo(20))
                    .splineToConstantHeading(new Vector2d(50, 36 * teamInvert), Math.toRadians(0), limitVelo(4));

            // place yellow pixel
            build = build.waitSeconds(1); // PLACEHOLDER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        } // end of params.placeYellowPixel

        // park on a side of the backstage
        // invert park direction: corner side is 1, center-field side is -1
        int parkInvert = 0;
        switch (params.parkLocation) {

            case CORNER_SIDE:
                parkInvert = 1;
                break;

            case CENTER_FIELD_SIDE:
                parkInvert = -1;
                break;
        }

        if (params.placeYellowPixel) {
            build = build.setTangent(Math.toRadians(180));
            build = build.splineToConstantHeading(new Vector2d(40, (36 + 12 * parkInvert) * teamInvert), Math.toRadians(90 * parkInvert * teamInvert), limitVelo(30))
                    .setTangent(Math.toRadians(90 * parkInvert * teamInvert));
        }

        // finish parking
        build = build.splineToConstantHeading(new Vector2d(50, (36 + 24 * parkInvert) * teamInvert), Math.toRadians(0), limitVelo(15));

        build = build.waitSeconds(1); // TEMPORARY FOR MEEPMEEP!!!!!!!!!

        // finish build
        return build.build();
    }

    /*
     * MeepMeep calls this routine to get a trajectory sequence action to draw. Modify the
     * arguments here to test your different code paths.
     */
    Action getMeepMeepAction() {

        AutoParams params = new AutoParams();
        params.allianceTeam = AutoParams.AllianceTeam.BLUE;
        params.startingPosition = AutoParams.StartingPosition.LONG;
        params.parkLocation = AutoParams.ParkLocation.CENTER_FIELD_SIDE;
        params.propPosition = ColorDetection.PropPosition.LEFT;
        params.placePurplePixel = true;
        params.placeYellowPixel = true;
        params.startLongWaitTime = 1;

        return getDriveAction(params);
    }

    private VelConstraint limitVelo(double limit) {
        return new VelConstraint() {
            @Override
            public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                return limit;
            }
        };
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
    public boolean placePurplePixel;
    public boolean placeYellowPixel;
    public double startLongWaitTime;

    public AutoParams() {}

}