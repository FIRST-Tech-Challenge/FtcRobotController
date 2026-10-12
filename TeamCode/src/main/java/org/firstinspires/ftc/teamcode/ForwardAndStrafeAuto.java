package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Forward 10cm Right 30cm", group="Robot")
public class ForwardAndStrafeAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        RockinBot robot = new RockinBot(this);

        telemetry.addData("Autonomous Ready", "Forward 10 cm, then right 30 cm");
        telemetry.addData("This code was last updated", "10/11/2026, 5:10 pm");
        telemetry.update();
        waitForStart();

        if (!opModeIsActive()) {
            return;
        }

        if (!robot.driveForwardMm(300)) {
            telemetry.addData("Autonomous", "Stopped: forward movement did not reach its target");
            telemetry.update();
            return;
        }

        if (!robot.strafeRightMm(600)) {
            telemetry.addData("Autonomous", "Stopped: right strafe did not reach its target");
            telemetry.update();
            return;
        }

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }
}
