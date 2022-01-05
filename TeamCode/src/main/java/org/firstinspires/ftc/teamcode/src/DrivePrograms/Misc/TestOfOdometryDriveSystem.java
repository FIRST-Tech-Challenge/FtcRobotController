package org.firstinspires.ftc.teamcode.src.DrivePrograms.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.Utills.AutonomousTemplate;
import org.firstinspires.ftc.teamcode.src.robotAttachments.DriveTrains.OdometryDrivetrain;

/**
 * A Autonomous test odometry
 */
@Autonomous(name = "TestOfOdometryDriveSystem")
public class TestOfOdometryDriveSystem extends AutonomousTemplate {

    public void runOpMode() throws InterruptedException {
        initAll();
        podServos.lower();
        waitForStart();
        odometry.setPosition(0, 0, 0);
        /*driveSystem.moveToPosition(10,10,0.5,true);
        driveSystem.moveToPosition(10,-10,0.5,true);
        driveSystem.moveToPosition(-10,-10,0.5,true);
        driveSystem.moveToPosition(-10,10,0.5,true);
        driveSystem.moveToPosition(0,0,0.5,true);
         */

        driveSystem.moveToPosition(0, 100, 0.5, true);

        for (int x = 100; x > 0; x -= 10) {
            driveSystem.move(OdometryDrivetrain.OdometryDirections.Backward, 10, .5);
        }

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Active", odometry.isActive());
            telemetry.addData("Thread State", odometry.isRunning());
            telemetry.addData("Thread State OBJ", odometry.getState());
            odometry.showPosition(telemetry);
            telemetry.update();
        }
    }
}
