package org.firstinspires.ftc.teamcode.src.DrivePrograms.Autonomous.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.Utills.AutonomousTemplate;


@Autonomous(name = "OdTurn")
@Disabled
public class OdTurn extends AutonomousTemplate {
    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();
        odometry.setPosition(0, 0, 0);
        telemetry.addData("Intialize Status", "complete");
        telemetry.update();

        waitForStart();
        driveSystem.turnTo(270, .5);
        driveSystem.stopAll();

    }

}
