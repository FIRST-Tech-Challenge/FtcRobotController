package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.base.GreenBotBaseOpMode;


@Autonomous(name = "Auton Drive Test", group = "Auton")
public class AutonDriveTest extends GreenBotBaseOpMode {

    static final double START_X = 63;
    static final double START_Y = -12;
    static final double START_HEADING = 180;

    static final double SPOT1_X = 53;
    static final double SPOT1_Y = -12;
    static final double SPOT1_HEADING = 180;

    static final double SPOT2_X = 53;
    static final double SPOT2_Y = -24;
    static final double SPOT2_HEADING = 180;

    static final double FAST_DRIVE_POWER = 1.0;
    static final double SLOW_DRIVE_POWER = 0.3;

    static final double LAUNCHER_TARGET_VELOCITY = 1250;

    @Override
    public void runOpMode() {
        config();

        telemetry.addData("Ready", "Press START");
        drive.showPose();

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y,
                AngleUnit.DEGREES, START_HEADING));

        waitForStart();

        drive.driveTo(SPOT1_X, SPOT1_Y, SPOT1_HEADING, SLOW_DRIVE_POWER, 0.8);

        drive.driveTo(SPOT2_X, SPOT2_Y, SPOT2_HEADING, SLOW_DRIVE_POWER, 0.8);

        drive.showPose();
        sleep(3000); // don't need this prolly
    }

}
