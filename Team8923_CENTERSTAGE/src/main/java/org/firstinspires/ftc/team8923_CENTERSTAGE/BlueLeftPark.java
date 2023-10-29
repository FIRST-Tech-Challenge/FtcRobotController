package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueLeftPark")

public class BlueLeftPark extends BaseAutonomous {
    @Override
    public void runOpMode() {
        initAuto();
        waitForStart();
        // move forward from blue alliance left side then strafe to backstage

        // move forward 4 inches to get off the wall
        driveInches(0, 4);
        // strafe left 62 inches to arrive in backstage
        driveInches(-62, 0);
    }
}
