package org.firstinspires.ftc.teamcode.toolkit.background;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.core.Background;

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
