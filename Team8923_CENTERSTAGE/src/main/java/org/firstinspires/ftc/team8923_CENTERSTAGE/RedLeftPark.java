package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedLeftPark")

public class RedLeftPark extends BaseAutonomous {
    @Override
    public void runOpMode() {
        initAuto();
        waitForStart();
        // go forward from red alliance left side then strafe to backstage

        // move forward 4 inches to get off the wall
        driveInches(0, 4);
        // strafe right 105 inches to get to the backstage
        driveInches(105, 0);
    }
}
