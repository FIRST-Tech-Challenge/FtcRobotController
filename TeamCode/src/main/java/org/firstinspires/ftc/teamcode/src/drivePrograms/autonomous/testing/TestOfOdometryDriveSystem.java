package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.robotAttachments.odometry.enums.OdometryDirections;
import org.firstinspires.ftc.teamcode.src.utills.AutonomousTemplate;

/**
 * A Autonomous test odometry
 */
@Disabled
@Autonomous(name = "TestOfOdometryDriveSystem")
public class TestOfOdometryDriveSystem extends AutonomousTemplate {

    public void opModeMain() throws InterruptedException {
        initAll();

        waitForStart();
        odometry.setPosition(0, 0, 0);

        driveSystem.moveToPosition(0, 100, 0.5);

        for (int x = 100; x > 0; x -= 10) {
            driveSystem.move(OdometryDirections.Backward, 10, .5);
        }

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Thread State", odometry.isRunning());
            telemetry.addData("Thread State OBJ", odometry.getState());
            odometry.showPosition(telemetry);
            telemetry.update();
        }
    }
}
