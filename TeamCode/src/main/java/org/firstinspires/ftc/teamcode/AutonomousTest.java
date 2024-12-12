package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.HangBot;

@Autonomous(name = "Auto Test", group = "Auto")
public class AutonomousTest extends LinearOpMode {
    protected HangBot robot = new HangBot(this);
    @Override
    public void runOpMode() throws InterruptedException {

        robot.isAuto = true;
        robot.init(hardwareMap);

        robot.pivotTarget = 300;

        while (!opModeIsActive()) {
            telemetry.addData("rotate position", robot.rotate.getPosition());
            telemetry.addData("slide position", robot.slideMotor.getCurrentPosition());
            telemetry.update();
        }

        waitForStart();

        robot.sleep(1000000);


//        robot.scoreBucket(true);
    }
}