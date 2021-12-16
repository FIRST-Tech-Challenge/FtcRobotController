package org.firstinspires.ftc.teamcode.src.DrivePrograms.Autonomous.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.Utills.AutonomousTemplate;


@Autonomous(name = "BlueAutonomousNearSpinner")
@Disabled
public class BlueAutonomousNearSpinner extends AutonomousTemplate {
    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();
        odometry.setPosition(133.5, 112, 180);
        telemetry.addData("Initialization Status: ", "Complete");
        telemetry.update();
        waitForStart();

        driveSystem.moveToPosition(126, 112, 1, true);
        driveSystem.moveToPosition(126, 132, 1.5, true);
        driveSystem.strafeAtAngle(180 + 35, .5);
        Thread.sleep(500);
        spinner.spinOffBlueDuck();
        driveSystem.moveToPosition(105, 132, 1);
        driveSystem.strafeAtAngle(180, 0.5);
        Thread.sleep(1000);


    }
}
