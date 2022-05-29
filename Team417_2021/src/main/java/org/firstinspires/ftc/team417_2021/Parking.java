package org.firstinspires.ftc.team417_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Parking")
public class Parking extends MasterAutonomous {

    int allianceSide = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        telemetry.addLine("ready");
        telemetry.update();
        while (!opModeIsActive()) {
            if (gamepad1.a) {
                allianceSide *= -1;
            }
            telemetry.addLine("1 for blue, -1 for red");
            telemetry.addData("Alliance", allianceSide);
            telemetry.update();
            idle();
        }

        waitForStart();
        moveInches(6, 0.8);
        pivot(90 * allianceSide, 0.8);
        moveInches(48, 1.0);
    }
}