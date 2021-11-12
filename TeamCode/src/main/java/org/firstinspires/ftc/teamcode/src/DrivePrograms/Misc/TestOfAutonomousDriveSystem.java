package org.firstinspires.ftc.teamcode.src.DrivePrograms.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.Utills.AutonomousTemplate;

@Disabled
@Autonomous(name = "TestOfAutonomousDriveSystem")
public class TestOfAutonomousDriveSystem extends AutonomousTemplate {

    public void runOpMode() throws InterruptedException {
        this.initAll();
        waitForStart();
        odometry.setPosition(0, 0, 0);
        driveSystem.moveToPosition(10, 10, 1, true);
        driveSystem.moveToPosition(-10, 10, 1, true);
        driveSystem.moveToPosition(-10, -10, 1, true);
        driveSystem.moveToPosition(10, -10, 1, true);
        driveSystem.moveToPosition(0, 0, 1, true);
        while (opModeIsActive() && !isStopRequested()) {
            odometry.showPosition(telemetry);
        }
    }
}
