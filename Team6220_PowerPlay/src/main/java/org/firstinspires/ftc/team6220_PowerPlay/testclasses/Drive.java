package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;

//@Disabled
@Autonomous(name = "Drive")
public class Drive extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        driveAutonomous(0, 24);

        sleep(1000);

        driveAutonomous(-90, 24);

        sleep(1000);

        driveAutonomous(180, 24);

        sleep(1000);

        driveAutonomous(90, 24);

        sleep(1000);
    }
}
