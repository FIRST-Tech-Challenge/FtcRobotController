package org.firstinspires.ftc.team417_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Comp")
public class AutonomousCompetition extends MasterAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        telemetry.addLine("Ready to Start");
        telemetry.update();
        waitForStart();
        pivot(-90, 0.5);
        move3(24, 0.5);
        pivot(0, 0.5);
        move3(-24, 0.5);
    }
}
