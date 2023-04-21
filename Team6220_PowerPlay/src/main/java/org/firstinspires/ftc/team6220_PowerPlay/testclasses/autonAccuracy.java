package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;

@Autonomous(name = "nav accuracy test")
public class autonAccuracy extends BaseAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        driveAutonomous(0, 20);
        sleep(500);
        driveAutonomous(90, 20);
        sleep(500);
        driveAutonomous(-90, 40);
        sleep(500);
        driveAutonomous(90, 20);
        sleep(500);
        driveAutonomous(180, 20);
    }
}
