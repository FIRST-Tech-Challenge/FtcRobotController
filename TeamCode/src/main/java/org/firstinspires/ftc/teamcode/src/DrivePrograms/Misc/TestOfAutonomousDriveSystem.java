package org.firstinspires.ftc.teamcode.src.DrivePrograms.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.Utills.AutonomousTemplate;

@Disabled
@Autonomous(name = "TestOfOdometryDriveSystem")
public class TestOfAutonomousDriveSystem extends AutonomousTemplate {

    public void runOpMode() throws InterruptedException {
        this.initAll();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            odometry.showPosition(telemetry);
        }
    }
}
