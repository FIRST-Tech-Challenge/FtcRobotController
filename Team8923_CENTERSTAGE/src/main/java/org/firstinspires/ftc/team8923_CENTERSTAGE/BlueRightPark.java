package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueRightPark")

public class BlueRightPark extends BaseAutonomous {
    @Override
    public void runOpMode() {
        initAuto();
        waitForStart();
        // move forward from blue alliance right side then strafe to backstage

        // move forward 4 inches to get off the wall
        driveInches(0, 4);
        // strafe left 107 inches to get to backstage
        driveInches(-107, 0);
    }
}
