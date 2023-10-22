package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AlternativeBlueRightPark")

public class AlternativeBlueRightPark extends BaseAutonomous {
    @Override
    public void runOpMode() {
        initAuto();
        waitForStart();
        // go forward then strafe to backstage
        driveInches(0, 3);
        driveInches(26, 0);
        driveInches(0, 48);
        driveInches(-142, 0);
    }
}
