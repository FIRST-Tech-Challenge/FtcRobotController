package org.firstinspires.ftc.teamcode.SpringLearning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "among_us_impostor_of_the_sus_variety")

public class GavinsSuspiciousMecanumTeleOp extends GavinBaseTeleopAMOGUS {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driveRobotUsingControllers();
        }
    }
}
