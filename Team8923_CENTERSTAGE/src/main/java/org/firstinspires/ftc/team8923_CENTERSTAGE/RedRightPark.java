package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedRightPark")

public class RedRightPark extends BaseAutonomous {
    @Override
    public void runOpMode() {
        initAuto();
        waitForStart();
        // go forward then strafe to backstage
        driveInches(0, 3);
        driveInches(30, 0);
    }
}
