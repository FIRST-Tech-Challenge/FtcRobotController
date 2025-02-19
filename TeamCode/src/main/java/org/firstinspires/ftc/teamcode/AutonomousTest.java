package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.FSMBot;
import org.firstinspires.ftc.teamcode.bots.HangBot;

@Autonomous(name = "Auto Test", group = "Auto")
public class AutonomousTest extends LinearOpMode {
    protected FSMBot robot = new FSMBot(this);
    @Override
    public void runOpMode() throws InterruptedException {

        robot.isAuto = true;
        robot.init(hardwareMap);

        while (!opModeIsActive()) {

            telemetry.update();
        }

        waitForStart();

        robot.sleep(1000000);


//        robot.scoreBucket(true);
    }}