package org.firstinspires.ftc.teamcode.src.DrivePrograms.Autonomous.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.Utills.AutonomousTemplate;


@Autonomous(name = "RedAutonomousNearSpinner")
@Disabled
public class RedAutonomousNearSpinner extends AutonomousTemplate {

    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();
        telemetry.addData("Initialization Status: ", "Complete");
        telemetry.update();
        waitForStart();
        odometry.setPosition(7.5, 111, 90);
        driveSystem.moveToPosition(17.5, 111, 1, true);
        driveSystem.moveToPosition(17.5, 134, 1, true);
        driveSystem.strafeAtAngle(235, 0.5);
        Thread.sleep(500);
        driveSystem.stopAll();
        spinner.spinOffRedDuck();
        driveSystem.moveToPosition(32, 134, 1, true);
        driveSystem.strafeAtAngle(270, 1);
        Thread.sleep(500);


    }
}
