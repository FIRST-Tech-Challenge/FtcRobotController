package org.firstinspires.ftc.team6220_2020;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous Competition", group = "Autonomous")
public class AutoCompetition extends MasterAutonomous{
    @Override
    public void runOpMode() throws InterruptedException {
        Initialize();

        waitForStart();

        driveForwardInches(24);
        pauseMillis(1000);
        driveForwardInches(-24);
    }
}
