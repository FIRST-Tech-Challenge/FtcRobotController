package org.firstinspires.ftc.team6220_2020;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous Competition", group = "Autonomous")
public class AutonomousCompetition extends MasterAutonomous {

    @Override
    public void runOpMode() {
        Initialize();
        waitForStart();

        for (int i = 0; i < 4; i++) {
            driveInches(20, 90);
            driveInches(20, 0);
            driveInches(20, -90);
            driveInches(20, 180);
        }

        //driveInches(20, 0);

        //driveInches(20, -90);

        //driveLauncher(0.8);

        //for(int i = 0; i < 3; i++){
        //    fireLauncher(1500);
        //    pauseMillis(1500);
        //}
    }
}