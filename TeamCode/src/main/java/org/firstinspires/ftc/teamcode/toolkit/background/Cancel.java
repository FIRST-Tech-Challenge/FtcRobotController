package org.firstinspires.ftc.teamcode.toolkit.background;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.UpliftRobot;

public class Cancel extends Background {

    UpliftRobot robot;

    public Cancel(UpliftRobot robot) {
        super(robot);
        this.robot = robot;
    }

    @Override
    public void loop() {
        robot.driverCancel = robot.opMode.gamepad1.dpad_left;
        robot.operatorCancel = robot.opMode.gamepad2.dpad_left;
    }

}
