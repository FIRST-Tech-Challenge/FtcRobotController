package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.src.utills.AutonomousTemplate;


/**
 * A OpMode to test Odometry turning capabilities
 */
@Autonomous(name = "OdTurn")
public class OdTurnTest extends AutonomousTemplate {
    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        odometry.setPosition(0, 0, 0);
        telemetry.addData("Intialize Status", "complete");
        telemetry.update();

        waitForStart();
        ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < 3000) {
            driveSystem.strafeAtAngleWhileTurn(0, 180, .8);
            checkStop();
        }
        driveSystem.stopAll();

    }

}
