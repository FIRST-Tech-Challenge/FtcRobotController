package org.firstinspires.ftc.team8923_PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoDetectPark")
public class AutoDetectPark extends BaseAutonomous {

    @Override
    public void runOpMode() {
        initAuto();
        waitForStart();
        driveInches(0, 2);
        switch (detectSignalSleeve()) {
            case ONE:
                // Strafe left one tile.
                driveInches(-27, 0);
                break;
            case TWO:
                // Already aligned for position two. Do nothing. :)
                break;
            case THREE:
                // Strafe right one tile.
                driveInches(27, 0);
                break;
        }
        driveInches(0, 30);
    }
}
