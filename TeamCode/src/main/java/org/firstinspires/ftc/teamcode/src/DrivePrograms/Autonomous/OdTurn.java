package org.firstinspires.ftc.teamcode.src.DrivePrograms.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.Utills.AutonomousTemplate;

@Autonomous(name = "OdTurn")
public class OdTurn extends AutonomousTemplate {
    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();
        odometry.setPosition(0, 0, 0);
        telemetry.addData("Intialize Status", "complete");
        telemetry.update();

        waitForStart();
        driveSystem.turnTo(90, .5);
        driveSystem.stopAll();

    }

}
