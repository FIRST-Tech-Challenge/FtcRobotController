package org.firstinspires.ftc.team417_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="test")
public class AutoTest extends MasterAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        telemetry.addLine("ready");
        telemetry.update();

        waitForStart();
        robot.setInitialAngle();

        pivot(90, 0.6);

    }
}
