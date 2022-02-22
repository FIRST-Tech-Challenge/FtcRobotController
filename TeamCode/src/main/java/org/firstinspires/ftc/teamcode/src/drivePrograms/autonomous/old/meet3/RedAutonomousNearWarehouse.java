package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.old.meet3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutonomousTemplate;

/**
 * The Autonomous ran on Red side near warehouse for Meet 3
 */
@Autonomous(name = "RedAutonomousNearWarehouse")
@Disabled
public class RedAutonomousNearWarehouse extends AutonomousTemplate {

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        telemetry.addData("Initialization Status: ", "Complete");
        telemetry.update();
        waitForStart();
        gps.setPos(7.5, 64, 180);
        driveSystem.moveToPosition(29, 64, 1);
        podServos.raise();
        driveSystem.strafeAtAngle(180, 1);
        Thread.sleep(50);
        driveSystem.stopAll();
        slide.setMotorPower(1);
        Thread.sleep(500);
        driveSystem.strafeAtAngle(0, 1);
        Thread.sleep(1250);
        driveSystem.stopAll();


    }
}
