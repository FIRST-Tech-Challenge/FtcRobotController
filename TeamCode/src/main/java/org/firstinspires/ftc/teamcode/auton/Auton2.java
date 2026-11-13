package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.base.GreenBotBaseOpMode;

@Autonomous(name = "Auton 2nd Shooter", group = "Auton")
public class Auton2 extends GreenBotBaseOpMode {

    static final double START_X = -63.5;
    static final double START_Y = -16;
    static final double START_HEADING = 180;

    static final double SHOOT_X = -60;
    static final double SHOOT_Y = -12;
    static final double SHOOT_HEADING = 180;

    static final double PARK_LEFT_X = -40;
    static final double PARK_LEFT_Y = -57;
    static final double PARK_HEADING = 180;

    static final double FAST_DRIVE_POWER = 1.0;
    static final double SLOW_DRIVE_POWER = 0.3;

    @Override
    public void runOpMode() {
        config();

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

        // drive to shooting spot
        drive.driveTo(SHOOT_X, SHOOT_Y, SHOOT_HEADING, SLOW_DRIVE_POWER);

        // wait for hive to tip -- need timeout in case it never does
        sleep(5000);

        // shoot all pollen
        sleep(2000);

        // park
        drive.driveTo(PARK_LEFT_X, PARK_LEFT_Y, PARK_HEADING, SLOW_DRIVE_POWER);
        sleep(5000);


        drive.showPose();
        sleep(3000);
    }
}
