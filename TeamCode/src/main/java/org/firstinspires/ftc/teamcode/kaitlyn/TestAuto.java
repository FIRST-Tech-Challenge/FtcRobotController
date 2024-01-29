package org.firstinspires.ftc.teamcode.kaitlyn;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry, true, true, false);
        robot.setUpDrivetrainMotors();
        //.setUpIntakeOuttake();
        robot.initVisionProcessing();

        waitForStart();

        while (opModeIsActive()) {
            robot.alignToBoardFast();
            //robot.mecanumBlocking(24,true,0.7);
            break;
        }
    }
}

