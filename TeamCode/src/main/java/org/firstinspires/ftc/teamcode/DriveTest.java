package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.FSMBot;

@Autonomous(name="Drive Test", group="Tests")

public class DriveTest extends LinearOpMode {

    protected FSMBot robot = new FSMBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        robot.driveStraightByTime(robot.DIRECTION_LEFT, 500, 0.5);
        robot.driveStraightByGyro(robot.DIRECTION_LEFT, 10, 0.5, false, 0, true);


//        int distanceFromStart = Math.abs(robot.horizontal.getCurrentPosition());
//        int drivingDistance = distanceFromStart + 27000;
//        robot.driveAgainstWallWithEncodersVertical(robot.DIRECTION_FORWARD, CameraBot.autoSide.BLUE, drivingDistance, 500, 0);
//        robot.drivingDone = true;
//        robot.setDropHeight(0);
//        robot.autoGrabFreight(0.2, CameraBot.autoSide.BLUE);

        //robot.driveAgainstWallWithEncodersVertical(robot.DIRECTION_FORWARD, robot.SIDE_BLUE, 40000, 500, 200);
    }
}
