package org.firstinspires.ftc.teamcode.competition.opmodes.templates;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOpTemplate", group="linear")
public class LinearTeleOpTemplate extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        resetStartTime();
    }

}
