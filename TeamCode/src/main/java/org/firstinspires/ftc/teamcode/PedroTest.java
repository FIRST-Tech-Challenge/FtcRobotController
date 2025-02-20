package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.bots.FSMBot;
import org.firstinspires.ftc.teamcode.bots.PedroPathingSampleBot;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class PedroTest extends LinearOpMode {
    protected PedroPathingSampleBot robot = new PedroPathingSampleBot(this);

    @Override
    public void runOpMode() throws InterruptedException {

        robot.isAuto = true;
        robot.init(hardwareMap);

        while (!opModeIsActive()) {

            telemetry.update();
        }

        waitForStart();

        robot.sleep(1000000);

    }
}