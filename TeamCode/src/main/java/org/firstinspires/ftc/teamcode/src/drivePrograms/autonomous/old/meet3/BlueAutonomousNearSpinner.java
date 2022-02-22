package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.old.meet3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutonomousTemplate;

/**
 * The Autonomous ran on Blue side near spinner for Meet 3
 */
@Autonomous(name = "BlueAutonomousNearSpinner")
@Disabled
public class BlueAutonomousNearSpinner extends AutonomousTemplate {
    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        driveSystem.debugOn();
        gps.setPos(133.5, 112, 180);
        telemetry.addData("Initialization Status: ", "Complete");
        telemetry.update();
        waitForStart();

        driveSystem.moveToPosition(126, 112, 1);
        driveSystem.moveToPosition(126, 132, 1.5);
        driveSystem.strafeAtAngle(180 + 35, .5);
        Thread.sleep(500);
        spinner.spinOffBlueDuck();
        driveSystem.moveToPosition(105, 132, 1);
        driveSystem.strafeAtAngle(180, 0.5);
        Thread.sleep(1000);


    }
}
