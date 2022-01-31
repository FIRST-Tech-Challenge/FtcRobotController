package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutonomousTemplate;


/**
 * A OpMode to test Odometry turning capabilities
 */
//@Disabled
@Autonomous(name = "OdTurn")
public class OdTurnTest extends AutonomousTemplate {
    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        odometry.setPos(0, 0, 0);
        telemetry.addData("Initialization Status", "complete");
        telemetry.update();

        waitForStart();

        driveSystem.newTurnToPrototype(20, 1, .1, 0);

        //driveSystem.stopAll();

    }

}
