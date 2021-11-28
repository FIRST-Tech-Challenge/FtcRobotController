package org.firstinspires.ftc.team417_2021;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "TeleOp Competition")
public class TeleOpCompetition extends MasterTeleOp {

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        telemetry.addLine("Ready for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driveRobotUsingController();
            controlMechanisms();
            robot.updatePosition();

            idle();
        }


    }
}
