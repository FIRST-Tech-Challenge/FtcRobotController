package org.firstinspires.ftc.team6220_2020;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous Competition", group = "Autonomous")
public class AutonomousCompetition extends MasterAutonomous {

    @Override
    public void runOpMode() {
        Initialize();
        waitForStart();

        driveInches(20, -90);

        driveInches(20, 0);

        driveInches(20, 90);

    }
}