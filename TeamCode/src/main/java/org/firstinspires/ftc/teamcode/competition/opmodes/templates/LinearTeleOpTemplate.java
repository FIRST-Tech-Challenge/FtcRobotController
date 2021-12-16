package org.firstinspires.ftc.teamcode.competition.opmodes.templates;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.competition.utils.scripting.ControlLoopManager;

@TeleOp(name="TeleOpTemplate", group="linear")
public class LinearTeleOpTemplate extends LinearOpMode {

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
