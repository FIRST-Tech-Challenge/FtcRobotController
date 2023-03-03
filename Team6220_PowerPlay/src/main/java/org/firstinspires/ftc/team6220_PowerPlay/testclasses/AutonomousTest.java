package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;

@Disabled
@Autonomous(name = "AutonomousTest")
public class AutonomousTest extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
    }
}