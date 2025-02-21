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

        while (opModeInInit()) {
            telemetry.addData("state", robot.currentState);
            telemetry.update();
            robot.updateTelemetry();
        }

        waitForStart();

        robot.currentState = FSMBot.gameState.DRIVE;

        robot.slideMotor1.setPower(0.6);
        robot.slideMotor2.setPower(0.6);
//        robot.driveToCoordinate(3000,-8000,45,1000,1,true); // scoring position
//        robot.waitForCoordinateDrive();

            robot.driveToCoordinate(4600, -4050, 45, 400, 0.3, true);// drive closer to bucket, bucket position

        telemetry.addData("Driving to coordinate" ,true);
        telemetry.update();
        robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_1; //raise pivot
//        robot.sleep(500);

        robot.waitForCoordinateDrive();
        telemetry.addData("Driving to coordinate" ,false);
        telemetry.update();
        robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_2; // raise slide
        robot.sleep(1500);
        robot.outtakeTimer.reset();
        robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_3; // deposit sample
        robot.sleep(1000);//sample is deposited and robot arm comes down, state resets to drive

            robot.driveToCoordinate(5468, -4050, 45, 400, 0.6, true); //drive to intake middle sample

        robot.waitForCoordinateDrive();

        robot.driveToCoordinate(5468, -8330, 0, 400, 0.6, true); //drive to intake middle sample

        robot.waitForCoordinateDrive();

        robot.subIntake(true);//sets gamestate to intake from submersible
        robot.sleep(500);


        robot.driveToCoordinate(5468, -8330, 45, 400, 0.1, true); //drive to intake middle sample



        robot.sleep(3000);

        robot.driveToCoordinate(4600, -4050, 45, 800, 0.8, true);// drive closer to bucket, bucket position
        robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_1; // raise pivot
        robot.waitForCoordinateDrive();
        robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_2; // raise slide
        robot.sleep(1500);
        robot.outtakeTimer.reset();
        robot.currentState = FSMBot.gameState.SAMPLE_SCORING_HIGH_3; // deposit sample
        robot.sleep(1000);//sample is deposited and robot arm comes down, state resets to drive

        /** run to intake 2nd sample*/

//        robot.driveToCoordinate(3000, -8000, -45, 500, 1, true); //drive to intake left sample
//        robot.subIntake(true);
//        robot.waitForCoordinateDrive();







    }
}