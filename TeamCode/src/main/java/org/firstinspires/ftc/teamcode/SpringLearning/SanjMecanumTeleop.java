package org.firstinspires.ftc.teamcode.SpringLearning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Sanj Mecanum Teleop")
public class SanjMecanumTeleop extends SanjBaseTeleop{
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
