package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutonomousTemplate;


/**
 * A OpMode to test Odometry turning capabilities
 */
@Disabled
@Autonomous(name = "OdTurn")
public class OdTurnTest extends AutonomousTemplate {
    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        initOdometryServos();
        gps.setPos(0, 0, 0);
        telemetry.addData("Initialization Status", "complete");
        telemetry.update();


        waitForStart();
        while (opModeIsActive()) {

            driveSystem.newTurnToPrototype(270, .7, .05, true);
        }

        telemetry.addData("done", "");
        telemetry.update();

        //driveSystem.stopAll();

    }

}
