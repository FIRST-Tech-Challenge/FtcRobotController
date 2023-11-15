package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AlternativeBlueLeftPark")

public class AlternativeBlueLeftPark extends BaseAutonomous {
    @Override
    public void runOpMode() {
        initAuto();
        waitForStart();
        // left one tile, forward three, left one tile
        driveInches(-24, 0);
        driveInches(0, 50);
        driveInches(-26, 0);
    }
}