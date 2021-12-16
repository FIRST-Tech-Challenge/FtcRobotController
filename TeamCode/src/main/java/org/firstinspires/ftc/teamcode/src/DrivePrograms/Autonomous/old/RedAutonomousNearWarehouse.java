package org.firstinspires.ftc.teamcode.src.DrivePrograms.Autonomous.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.Utills.AutonomousTemplate;

@Autonomous(name = "RedAutonomousNearWarehouse")
@Disabled
public class RedAutonomousNearWarehouse extends AutonomousTemplate {

    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();
        telemetry.addData("Initialization Status: ", "Complete");
        telemetry.update();
        waitForStart();
        odometry.setPosition(7.5, 64, 180);
        driveSystem.moveToPosition(29, 64, 1, true);
        podServos.raise();
        driveSystem.strafeAtAngle(180, 1);
        Thread.sleep(50);
        driveSystem.stopAll();
        slide.setMotorPower(1);
        Thread.sleep(500);
        slide.end();
        driveSystem.strafeAtAngle(0, 1);
        Thread.sleep(1250);
        driveSystem.stopAll();


    }
}
