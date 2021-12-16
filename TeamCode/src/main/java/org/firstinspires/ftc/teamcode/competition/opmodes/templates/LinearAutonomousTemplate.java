package org.firstinspires.ftc.teamcode.competition.opmodes.templates;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.competition.utils.scripting.ControlLoopManager;

@Autonomous(name="AutonomousTemplate", group="linear")
public class LinearAutonomousTemplate extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // CONSTRUCT script HERE
        ControlLoopManager loopManager = new ControlLoopManager(this);
        waitForStart();
        resetStartTime();
        while(loopManager.shouldContinue()) {
            // RUN script.main() HERE
        }
        // RUN script.stop() HERE
    }

}
