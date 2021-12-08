package org.firstinspires.ftc.team6220_2021.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.team6220_2021.MasterAutonomous;

@Autonomous(name = "Test", group = "Test")
public class Test extends MasterAutonomous {

    @Override
    public void runOpMode() {
        Initialize();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                driveInches(24, 0.75, true);
                pauseMillis(2000);
                driveInches(24, 0.75, false);
                pauseMillis(2000);
                turnDegrees(90);
                pauseMillis(2000);
                turnDegrees(-90);
                break;
            }
        }
    }
}