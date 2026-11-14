package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.base.GreenBotBaseOpMode;


@Autonomous(name = "Auton 1st Shooter", group = "Auton")
public class Auton1 extends GreenBotBaseOpMode {

    static final double START_X = 63;
    static final double START_Y = -18;
    static final double START_HEADING = 180;

    static final double GARDEN_START_X = 47;
    static final double GARDEN_END_X = 62.5;
    static final double GARDEN_Y = -50;
    static final double GARDEN_HEADING = 0;

    static final double MIDPOINT_X = -12.0;
    static final double MIDPOINT_Y = -48.0;
    static final double MIDPOINT_HEADING = 0;

    static final double SHOOT2_X = -52.5;
    static final double SHOOT2_Y = -16;
    static final double SHOOT2_HEADING = 180;

    static final double PARK_RIGHT_X = -30;
    static final double PARK_RIGHT_Y = -50;
    static final double PARK_HEADING = 180;

    static final double FAST_DRIVE_POWER = 1.0;
    static final double SLOW_DRIVE_POWER = 0.3;

    static final double LAUNCHER_TARGET_VELOCITY = 1250;

    @Override
    public void runOpMode() {
        config();
        drive.setUsePid(true);

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y,
                AngleUnit.DEGREES, START_HEADING));

        // Pick the alliance during INIT; the LEDs show the alliance color until START.
        while (opModeInInit()) {
            selectAlliance();
            telemetry.addData("Ready", "Press START");
            drive.showPose();
            telemetry.update();
        }

        waitForStart();
        wedgeServo.setPosition(WEDGE_UP);
        aimer.start(); // so the aimer LED shows whether the hive is in view
        launcher.setTargetVelocity(LAUNCHER_TARGET_VELOCITY);

        waitForLauncher(2.0); // feed as soon as it's up to speed (gives up after 2 s)
        feeder.setPower(1.0);
        sleep(3000);
        feeder.setPower(0);

        // pick up pollen from the garden
        drive.driveTo(GARDEN_START_X, GARDEN_Y, START_HEADING, FAST_DRIVE_POWER,0.8);
        sleep(500);
        drive.driveTo(GARDEN_END_X, GARDEN_Y, GARDEN_HEADING, SLOW_DRIVE_POWER);
        sleep(500);

        // drive to 2nd shooting spot
        drive.driveTo(MIDPOINT_X, MIDPOINT_Y, MIDPOINT_HEADING, FAST_DRIVE_POWER, 0.8, 8.0);
        drive.showPose();

        sleep(2000);
        drive.driveTo(SHOOT2_X, SHOOT2_Y, SHOOT2_HEADING, FAST_DRIVE_POWER, 0.8, 8.0);
        drive.showPose();

        // wait for hive, need timeout if hive never tips
        sleep(1000);
        // aim and shoot - only if hive shows up
        sleep(1000);

        // park
        drive.driveTo(PARK_RIGHT_X, PARK_RIGHT_Y,PARK_HEADING, FAST_DRIVE_POWER);
        sleep(1000);

        drive.showPose();
        sleep(3000); // don't need this prolly
    }
}
