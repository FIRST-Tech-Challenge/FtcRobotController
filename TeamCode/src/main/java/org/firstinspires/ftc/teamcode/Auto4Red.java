package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.CameraBot;
import org.firstinspires.ftc.teamcode.bots.SnarmBot;

@Autonomous(name="Auto 4 (RED Warehouse)", group="Autos")

public class Auto4Red extends LinearOpMode {

    protected CameraBot robot = new CameraBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.snarmSnarmState = 0;
        robot.side = CameraBot.autoSide.RED;
        waitForStart();
        robot.isAutoStart = true;
        int[] pos;
        pos = robot.detect();
        robot.setDropHeight(pos[0]);
        if (false) {
            robot.setSnarmRotation(0);
        } else {
            robot.setSnarmRotation(1);
        }

        robot.waitOnSnarmState(SnarmBot.SnarmState.RELEASING, 10000);
        int distanceFromStart = Math.abs(robot.horizontal.getCurrentPosition());
        int drivingDistance = distanceFromStart + 27000;
        robot.driveAgainstWallWithEncodersVertical(robot.DIRECTION_FORWARD, robot.side, drivingDistance, 500, 0);
        robot.drivingDone = true;
        robot.setDropHeight(0);
        int warehouseDepletion = Math.abs(robot.autoGrabFreight(false, robot.side, robot.side, true));

        robot.waitOnSnarmState(SnarmBot.SnarmState.RELEASING, 10000);
        distanceFromStart = Math.abs(robot.horizontal.getCurrentPosition());
        drivingDistance = distanceFromStart + warehouseDepletion - 9000;
        robot.driveAgainstWallWithEncodersVertical(robot.DIRECTION_FORWARD, robot.side, drivingDistance, 500, 0);
        robot.drivingDone = true;

        warehouseDepletion = Math.abs(robot.autoGrabFreight(false, robot.side, robot.side, true));
        robot.setDropHeight(0);

        robot.waitOnSnarmState(SnarmBot.SnarmState.RELEASING, 10000);
        distanceFromStart = Math.abs(robot.horizontal.getCurrentPosition());
        drivingDistance = distanceFromStart + warehouseDepletion - 11000;
        robot.driveAgainstWallWithEncodersVertical(robot.DIRECTION_FORWARD, robot.side, drivingDistance, 500, 0);
        robot.drivingDone = true;

        warehouseDepletion = Math.abs(robot.autoGrabFreight(false, CameraBot.autoSide.BLUE, robot.side, true));
        robot.setDropHeight(0);

        robot.waitOnSnarmState(SnarmBot.SnarmState.RELEASING, 10000);
        distanceFromStart = Math.abs(robot.horizontal.getCurrentPosition());
        drivingDistance = distanceFromStart + warehouseDepletion - 14000;
        robot.driveAgainstWallWithEncodersVertical(robot.DIRECTION_FORWARD, robot.side, drivingDistance, 500, 0);
        robot.drivingDone = true;

        warehouseDepletion = Math.abs(robot.autoGrabFreight(false, CameraBot.autoSide.BLUE, robot.side, false));
        robot.sleep(2000);

    }
}
