package org.firstinspires.ftc.teamcode.src.DrivePrograms.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.Utills.AutonomousTemplate;

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
