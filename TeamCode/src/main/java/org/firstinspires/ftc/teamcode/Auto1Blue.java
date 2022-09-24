package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.CameraBot;
import org.firstinspires.ftc.teamcode.bots.SnarmBot;

@Autonomous(name="Auto 1 (BLUE Duck)", group="Autos")

public class Auto1Blue extends LinearOpMode {

    protected CameraBot robot = new CameraBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.odometryRaise.setPosition(0.65);
        robot.snarmSnarmState = 0;
        waitForStart();
        robot.isAutoStart = true;
        int[] pos;
        pos = robot.detect();
        if (true) {
            robot.setSnarmRotation(1);
        } else {
            robot.setSnarmRotation(0);
        }
        robot.setDropHeight(pos[0]);

        robot.drivingDone = true;
        robot.waitOnSnarmState(SnarmBot.SnarmState.RAISING_INTAKE, 10000);
        robot.snarmState = SnarmBot.SnarmState.IDLE;
        //robot.driveStraightByTime(robot.DIRECTION_LEFT, 1000, 0.3);
        //robot.goToAnglePID(0);
        robot.driveStraightByGyro(robot.DIRECTION_LEFT, 10, 0.5, false, 0, true);

        robot.sleep(200);
        robot.driveStraightByGyro(robot.DIRECTION_FORWARD, 9, 0.2, false, 0, true);
        robot.driveStraightByTime(robot.DIRECTION_FORWARD, 500, 0.2);
        robot.sleep(200);
        robot.driveStraightByTime(robot.DIRECTION_RIGHT, 1800, 0.2);
        robot.driveStraightByDistance(robot.DIRECTION_LEFT, 0.2, 0.14);
        robot.toggleSpinner(0.27, false);
        robot.sleep(3500);
        robot.toggleSpinner(0.27, false);
        //robot.goToAnglePID(0);
        robot.sleep(200);
        robot.driveStraightByGyro(robot.DIRECTION_LEFT, 15, 0.5, false, 0, true);
    }
}
