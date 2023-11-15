package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AlternativeRedLeftPark")

public class AlternativeRedLeftPark extends BaseAutonomous {
    @Override
    public void runOpMode() {
        initAuto();
        waitForStart();
        // move forward (to not be against wall), left one tile, forward three, left six
        driveInches(0, 3);
        driveInches(-26, 0);
        driveInches(0, 54);
        driveInches(142, 0);
    }
}