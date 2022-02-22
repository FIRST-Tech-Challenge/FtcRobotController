package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.old.meet3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutonomousTemplate;

/**
 * The Autonomous ran on Blue side near warehouse for Meet 3
 */
@Autonomous(name = "BlueAutonomousNearWarehouse")
@Disabled
public class BlueAutonomousNearWarehouse extends AutonomousTemplate {

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        gps.setPos(133.5, 64, 180);
        telemetry.addData("Initialization Status: ", "Complete");
        telemetry.update();
        waitForStart();

        driveSystem.moveToPosition(106, 65, 2);
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