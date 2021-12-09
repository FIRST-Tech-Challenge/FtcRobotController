package org.firstinspires.ftc.teamcode.competition.opmodes.templates;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutonomousTemplate", group="linear")
public class LinearAutonomousTemplate extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        resetStartTime();
    }

}
