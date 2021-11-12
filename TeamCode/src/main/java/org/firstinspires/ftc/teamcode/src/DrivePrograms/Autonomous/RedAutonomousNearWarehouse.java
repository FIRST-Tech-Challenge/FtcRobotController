package org.firstinspires.ftc.teamcode.src.DrivePrograms.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.Utills.AutonomousTemplate;

@Autonomous(name = "RedAutonomousNearWarehouse")
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
        slide.stop();
        driveSystem.strafeAtAngle(0, 1);
        Thread.sleep(1250);
        driveSystem.stopAll();


    }
}
