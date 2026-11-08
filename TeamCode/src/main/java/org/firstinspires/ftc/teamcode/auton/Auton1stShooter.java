package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.base.GreenBotBaseOpMode;
import org.firstinspires.ftc.teamcode.base.OrangeBotBaseOpMode;


@Autonomous(name = "Auton 1st Shooter", group = "Auton")
public class Auton1stShooter extends GreenBotBaseOpMode {

    static final double START_X = 63;
    static final double START_Y = -12;
    static final double START_HEADING = 180;

    static final double GARDEN_START_X = 47;
    static final double GARDEN_END_X = 58;
    static final double GARDEN_Y = -56;
    static final double GARDEN_HEADING = 0;

    static final double SHOOT2_X = -52.5;
    static final double SHOOT2_Y = -12;
    static final double SHOOT2_HEADING = 0;

    static final double PARK_RIGHT_X = -20;
    static final double PARK_RIGHT_Y = -57;
    static final double PARK_HEADING = 90;

    static final double FAST_DRIVE_POWER = 1.0;
    static final double SLOW_DRIVE_POWER = 0.3;

    @Override
    public void runOpMode() {
        config();

        telemetry.addData("Ready", "Press START");
        drive.showPose();

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y,
                AngleUnit.DEGREES, START_HEADING));

        waitForStart();

        launcher.setPower(1.0);

        sleep(2000);
        feeder.setPower(1.0);
        sleep(1000);
        feeder.setPower(0);

        // pick up pollen from the garden
        drive.driveTo(GARDEN_START_X, GARDEN_Y, GARDEN_HEADING, FAST_DRIVE_POWER,0.8);
        sleep(500);
        drive.driveTo(GARDEN_END_X, GARDEN_Y, GARDEN_HEADING, SLOW_DRIVE_POWER);
        sleep(500);

        // drive to 2nd shooting spot
        drive.driveTo(SHOOT2_X, SHOOT2_Y, SHOOT2_HEADING, FAST_DRIVE_POWER);

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
