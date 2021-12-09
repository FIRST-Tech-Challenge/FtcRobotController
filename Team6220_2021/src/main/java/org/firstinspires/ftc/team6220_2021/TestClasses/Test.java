package org.firstinspires.ftc.team6220_2021.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team6220_2021.MasterAutonomous;

@Disabled
@Autonomous(name = "Test", group = "Test")
public class Test extends MasterAutonomous {

    @Override
    public void runOpMode() {
        Initialize();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
<<<<<<< HEAD
//                driveInches(24, 0.75, true);
//                pauseMillis(2000);
//                driveInches(24, 0.75, false);
//                pauseMillis(2000);
//                turnDegrees(90);
//                pauseMillis(2000);
//                turnDegrees(-90);

                driveTank(0.5, 0.5);
=======
                driveInches(24, true);
                pauseMillis(2000);
                driveInches(24, false);
                pauseMillis(2000);
                turnDegrees(90);
                pauseMillis(2000);
                turnDegrees(-90);
>>>>>>> c8b3d9893abf9fb4cae33c73a63fec30dfd14d36
                break;
            }
        }
    }
}