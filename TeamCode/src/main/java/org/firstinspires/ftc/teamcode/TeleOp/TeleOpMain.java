package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Config
public class TeleOpMain extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Intake intake = new Intake(this);

        waitForStart();
        intake.teleOp();
    }
}
