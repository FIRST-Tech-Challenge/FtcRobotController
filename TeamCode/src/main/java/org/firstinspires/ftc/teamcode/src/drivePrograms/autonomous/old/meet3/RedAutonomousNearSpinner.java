package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.old.meet3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutonomousTemplate;

/**
 * The Autonomous ran on Red side near spinner for Meet 3
 */
@Autonomous(name = "RedAutonomousNearSpinner")
@Disabled
public class RedAutonomousNearSpinner extends AutonomousTemplate {

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        telemetry.addData("Initialization Status: ", "Complete");
        telemetry.update();
        waitForStart();
        gps.setPos(7.5, 111, 90);
        driveSystem.moveToPosition(17.5, 111, 1);
        driveSystem.moveToPosition(17.5, 134, 1);
        driveSystem.strafeAtAngle(235, 0.5);
        Thread.sleep(500);
        driveSystem.stopAll();
        spinner.spinOffRedDuck();
        driveSystem.moveToPosition(32, 134, 1);
        driveSystem.strafeAtAngle(270, 1);
        Thread.sleep(500);


    }
}
