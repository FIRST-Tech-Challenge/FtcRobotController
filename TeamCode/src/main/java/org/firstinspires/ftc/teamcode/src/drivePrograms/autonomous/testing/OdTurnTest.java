package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.utills.AutonomousTemplate;


/**
 * A OpMode to test Odometry turning capabilities
 */
@Autonomous(name = "OdTurn")
@Disabled
public class OdTurnTest extends AutonomousTemplate {
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
