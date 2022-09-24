package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.CameraBot;

@Autonomous(name="Camera Test", group="Tests")

public class CameraTest extends LinearOpMode {

    protected CameraBot robot = new CameraBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        int[] pos = robot.detect();
        telemetry.addData("duck:", pos[0]);
        telemetry.addData("side:", pos[1]);
        telemetry.update();
        robot.sleep(15000);
    }

}
