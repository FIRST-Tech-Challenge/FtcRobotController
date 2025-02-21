package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.FSMBot;
import org.firstinspires.ftc.teamcode.bots.OdometryBot;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "Sample Auto", group = "Auto")
public class AutonomousV1 extends LinearOpMode {
    protected FSMBot robot = new FSMBot(this);
    @Override
    public void runOpMode() throws InterruptedException {

        robot.isAuto = true;
        robot.init(hardwareMap);

        while (!opModeIsActive()) {
            telemetry.addData("state", robot.currentState);
            telemetry.update();
        }

        waitForStart();

        robot.currentState = FSMBot.gameState.DRIVE;

        robot.slideMotor1.setPower(0.6);
        robot.slideMotor2.setPower(0.6);
        robot.driveToCoordinate(3000,-8000,45,1000,1,true);
        robot.waitForCoordinateDrive();
        robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_1;
        robot.sleep(500);

        robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_2;
        robot.sleep(1000);

        robot.driveToCoordinate(2500, -5500, 45, 700, 0.8, true);
        robot.waitForCoordinateDrive();
        robot.outtakeTimer.reset();
        robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_3;
        robot.sleep(1000);


    }
}