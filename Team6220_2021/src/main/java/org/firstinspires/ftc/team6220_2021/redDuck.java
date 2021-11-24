package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Red Duck", group = "Autonomous")
@Disabled
public class redDuck extends MasterAutonomous {

    @Override
    public void runOpMode() {
        Initialize();
        waitForStart();
        pauseMillis(500);
        driveInches(36, 0.5);
        turnDegrees(-135, 0.6);
        driveInches(40, 0.8);
        driveInches(1, 0.3);
        redDuckTask();
        driveInches(-21, 0.5);
        turnDegrees(40, 0.6);
        driveInches(25, 0.7);
    }
}