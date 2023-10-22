package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AlternativeRedRightPark")

public class AlternativeRedRightPark extends BaseAutonomous {
    @Override
    public void runOpMode() {
        initAuto();
        waitForStart();
        // go forward then strafe to backstage
        driveInches(24, 0);
        driveInches(0, 54);
        driveInches(26, 0);
    }
}