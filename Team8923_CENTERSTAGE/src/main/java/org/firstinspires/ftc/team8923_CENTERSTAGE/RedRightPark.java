package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedRightPark")

public class RedRightPark extends BaseAutonomous {
    @Override
    public void runOpMode() {
        initAuto();
        waitForStart();
        // move forward from red alliance right side then strafe to backstage

        // move forward 4 inches to get off the wall
        driveInches(0, 4);
        // strafe right 61 inches to get to backstage
        driveInches(61, 0);
    }
}
